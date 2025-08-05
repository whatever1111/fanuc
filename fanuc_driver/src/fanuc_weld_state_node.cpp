#include <ros/ros.h>
#include <fanuc_driver/WeldState.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>
#include <vector>
#include <cctype>

// Scaling factors from EWM manual
const double VOLTAGE_SCALE = 100.0 / 32767.0;   // Raw to Volts
const double CURRENT_SCALE = 1000.0 / 32767.0;  // Raw to Amperes
const double WIRE_SPEED_SCALE = 40.0 / 32767.0;  // Raw to m/min

class FanucWeldStateNodeTcp
{
private:
  ros::NodeHandle nh_;
  ros::Publisher weld_state_pub_;
  std::string robot_ip_;
  int robot_port_;
  int sock_fd_;
  bool debug_;
  bool little_endian_;   // true: expect little-endian payload, false: big-endian
  
  // Protocol parameters (loaded from ROS params / launch file)
  uint32_t expected_standard_length_;   // e.g. 61
  bool dump_raw_;                       // print full raw message in hex
  uint32_t expected_msg_type_;          // e.g. 15
  uint32_t expected_comm_type_;         // e.g. 1 (RI_CT_TOPIC or SVCREQ depending on sender)
  int      payload_fields_;             // number of Int32 values in payload (default 9)
  std::size_t payload_size_bytes_;      // derived: payload_fields_ * 4
  
public:
  FanucWeldStateNodeTcp() : nh_("~"), sock_fd_(-1)
  {
    // Get parameters
    nh_.param<std::string>("robot_ip", robot_ip_, "127.0.0.1");
    nh_.param<int>("robot_port", robot_port_, 11002);
    nh_.param<bool>("debug", debug_, false);

    // Protocol tuning parameters (can be set from launch file to avoid hard-coded magic numbers)
    int tmp_int = 0;
    nh_.param<int>("expected_standard_length", tmp_int, 53);
    expected_standard_length_ = static_cast<uint32_t>(tmp_int);
    nh_.param<int>("expected_msg_type", tmp_int, 15);
    expected_msg_type_ = static_cast<uint32_t>(tmp_int);
    nh_.param<int>("expected_comm_type", tmp_int, 1);
    expected_comm_type_ = static_cast<uint32_t>(tmp_int);
    nh_.param<int>("payload_fields", payload_fields_, 9);
    payload_size_bytes_ = static_cast<std::size_t>(payload_fields_) * sizeof(int32_t);
    nh_.param<bool>("dump_raw", dump_raw_, false);

    // Byte order parameter: "little" (default) or "big"
    std::string byte_order_param;
    nh_.param<std::string>("byte_order", byte_order_param, std::string("little"));
    std::transform(byte_order_param.begin(), byte_order_param.end(), byte_order_param.begin(), ::tolower);
    little_endian_ = (byte_order_param == "little" || byte_order_param == "le");
    
    // Setup publisher
    weld_state_pub_ = nh_.advertise<fanuc_driver::WeldState>("weld_state", 10);
    
    ROS_INFO("Fanuc Weld State TCP Node Starting");
    ROS_INFO("Target: %s:%d", robot_ip_.c_str(), robot_port_);
    if (debug_) ROS_INFO("Debug mode enabled");
    ROS_INFO("Expecting %s-endian robot messages", little_endian_ ? "little" : "big");
  }
  
  ~FanucWeldStateNodeTcp()
  {
    if (sock_fd_ >= 0)
    {
      close(sock_fd_);
    }
  }

  bool init()
  {
    ROS_INFO("Connecting to KAREL program...");
    
    // Create socket
    sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0)
    {
      ROS_ERROR("Failed to create socket");
      return false;
    }
    
    // Setup server address
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    // Explicitly cast to uint16_t to avoid implicit narrowing warning
    server_addr.sin_port = htons(static_cast<uint16_t>(robot_port_));
    
    if (inet_pton(AF_INET, robot_ip_.c_str(), &server_addr.sin_addr) <= 0)
    {
      ROS_ERROR("Invalid IP address: %s", robot_ip_.c_str());
      close(sock_fd_);
      sock_fd_ = -1;
      return false;
    }
    
    // Connect to server
    if (connect(sock_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
    {
      ROS_ERROR("Failed to connect to %s:%d", robot_ip_.c_str(), robot_port_);
      ROS_ERROR("Make sure KAREL program is running and cfg_.checked = TRUE");
      close(sock_fd_);
      sock_fd_ = -1;
      return false;
    }
    
    ROS_INFO("Successfully connected to KAREL program");
    return true;
  }
  
  void run()
  {
    int msg_count = 0;
    int error_count = 0;
    
    ROS_INFO("Starting weld state monitoring...");
    
    while (ros::ok())
    {
      // Receive header (16 bytes: length, msg_type, comm_type, reply_type)
      uint8_t header_buf[16];
      if (!receiveExactly(header_buf, 16))
      {
        error_count++;
        if (error_count % 10 == 1)
        {
          ROS_WARN("Communication error (count: %d)", error_count);
        }
        
        // Try to reconnect
        if (error_count > 5 && !reconnect())
        {
          ROS_ERROR("Failed to reconnect, exiting");
          break;
        }
        continue;
      }
      
      // Helper lambdas for endian-aware reads
      auto readUint32 = [this](const uint8_t* data) -> uint32_t
      {
        uint32_t val;
        std::memcpy(&val, data, sizeof(uint32_t));
        if (little_endian_)
          return val;                // already little-endian on x86 host
        else
          return ntohl(val);         // convert big-endian (network) to host
      };

      auto readInt32 = [this](const uint8_t* data) -> int32_t
      {
        uint32_t tmp;
        std::memcpy(&tmp, data, sizeof(uint32_t));
        if (!little_endian_)
          tmp = ntohl(tmp);
        return static_cast<int32_t>(tmp);
      };

      uint32_t length     = readUint32(&header_buf[0]);
      uint32_t msg_type   = readUint32(&header_buf[4]);
      uint32_t comm_type  = readUint32(&header_buf[8]);
      uint32_t reply_type = readUint32(&header_buf[12]);
      
      msg_count++;
      if (debug_)
      {
        ROS_INFO("Message #%d - Length: %u, Type: %u, Comm: %u, Reply: %u", 
                 msg_count, length, msg_type, comm_type, reply_type);
      }

      // Dump raw message bytes if requested
      if (dump_raw_)
      {
        std::vector<uint8_t> raw(16 + length);
        std::memcpy(raw.data(), header_buf, 16);
        // peek remaining bytes without removing from socket
        ssize_t peeked = recv(sock_fd_, raw.data() + 16, length, MSG_PEEK);
        if (peeked > 0)
        {
          std::ostringstream oss;
          oss << "RAW(" << (16 + peeked) << "):";
          for (size_t i = 0; i < 16 + (size_t)peeked; ++i)
          {
            oss << ' ' << std::hex << std::setw(2) << std::setfill('0') << int(raw[i]);
          }
          ROS_INFO_STREAM(oss.str());
        }
      }
      
      // Basic header validation using configurable parameters
      if (msg_type != expected_msg_type_ || comm_type != expected_comm_type_ ||
          length < (4 + payload_size_bytes_))
      {
        ROS_WARN("Invalid header - Expect: msg_type=%u, comm_type=%u, length>=%zu", 
                 expected_msg_type_, expected_comm_type_, 4 + payload_size_bytes_);
        if (debug_)
        {
          ROS_WARN("Received: Length=%u, Type=%u, Comm=%u, Reply=%u", length, msg_type, comm_type, reply_type);
        }
        // consume and discard the reported length bytes to realign stream
        if (length > 0)
        {
          std::vector<uint8_t> junk(length);
          receiveExactly(junk.data(), length);
        }
        continue;
      }
      
      // Receive the remaining <length> bytes of this message
      // 'length' counts everything AFTER this 4-byte field, which includes
      // msg_type, comm_type, reply_type (already read: 12 bytes).
      if (length < 12)
      {
        ROS_WARN("Length field too small (%u) – skipping", length);
        continue;
      }
      uint32_t remaining_bytes = length - 12;  // bytes still to read
      std::vector<uint8_t> remainder(remaining_bytes);
      if (!receiveExactly(remainder.data(), remaining_bytes))
      {
        ROS_ERROR("Failed to receive remaining %u bytes", remaining_bytes);
        continue;
      }

      // Analyse remainder for optional seq_nr (4B) and at most one trailing CR (0x0d or 0x0a)
      // Acceptable data sizes after trimming CR/LF:
      // 36  = payload only (9×INT)
      // 40  = seq(4)+payload
      // If there is a single trailing CR (0x0d or 0x0a) add +1 → 37 / 41

      std::size_t payload_offset = 0;
      uint32_t seq_nr = 0;

      // First remove max 2 trailing CR/LF bytes (not counted in length field on some robots)
      std::size_t trim_bytes = 0;
      while (trim_bytes < 2 && !remainder.empty())
      {
        uint8_t last = remainder[remainder.size() - 1 - trim_bytes];
        if (last == '\r' || last == '\n')
          ++trim_bytes;
        else
          break;
      }
      std::size_t data_bytes = remaining_bytes - trim_bytes;

      if (data_bytes == payload_size_bytes_ || data_bytes == payload_size_bytes_ + 1) // payload (+optional CR)
      {
        payload_offset = 0;
      }
      else if (data_bytes == payload_size_bytes_ + 4 || data_bytes == payload_size_bytes_ + 5) // seq + payload (+optional CR)
      {
        seq_nr = readUint32(&remainder[0]);
        payload_offset = 4;
      }
      else
      {
        ROS_WARN("Unexpected data bytes %zu (accept %zu/%zu/%zu/%zu) – skipping", data_bytes,
                 payload_size_bytes_, payload_size_bytes_ + 1,
                 payload_size_bytes_ + 4, payload_size_bytes_ + 5);
        continue;
      }

      const uint8_t* payload_ptr = &remainder[payload_offset];
      std::size_t available_payload_bytes = data_bytes - payload_offset;

      // Parse payload_fields_ ints
      std::size_t ints_in_msg = available_payload_bytes / 4;
      if (ints_in_msg < static_cast<std::size_t>(payload_fields_))
      {
        ROS_WARN("Payload too small (%zu ints) – skipping", ints_in_msg);
        continue;
      }
      std::vector<int32_t> weld_ints(payload_fields_);
      for (int i = 0; i < payload_fields_; ++i)
        weld_ints[i] = readInt32(payload_ptr + i * 4);

      // consume any leftover CR/LF still in socket (not counted in length)
      while (true)
      {
        uint8_t peek;
        ssize_t r = recv(sock_fd_, &peek, 1, MSG_PEEK | MSG_DONTWAIT);
        if (r == 1 && (peek == '\r' || peek == '\n'))
          recv(sock_fd_, &peek, 1, 0); // discard
        else
          break;
      }
      

      
      if (debug_)
      {
        ROS_INFO("Seq: %u, Raw weld data:", seq_nr);
        ROS_INFO("  arc_ok: %d, power_err: %d, depos_di: %d", 
                 weld_ints[0], weld_ints[1], weld_ints[2]);
        ROS_INFO("  ewm_err: %d, warn: %d, voltage: %d, current: %d, wire_spd: %d, motor_curr: %d",
                 weld_ints[3], weld_ints[4], weld_ints[5], weld_ints[6], weld_ints[7], weld_ints[8]);
      }
      
      // Create ROS message
      fanuc_driver::WeldState weld_msg;
      weld_msg.arc_ok        = (weld_ints[0] != 0);
      weld_msg.power_err     = (weld_ints[1] != 0);
      weld_msg.depos_di      = (weld_ints[2] != 0);
      weld_msg.ewm_err       = static_cast<int16_t>(weld_ints[3]);
      weld_msg.warning_state = static_cast<int16_t>(weld_ints[4]);
      weld_msg.act_voltage   = static_cast<int16_t>(weld_ints[5]);
      weld_msg.act_current   = static_cast<int16_t>(weld_ints[6]);
      weld_msg.act_wire_spd  = static_cast<int16_t>(weld_ints[7]);
      weld_msg.motor_current = static_cast<int16_t>(weld_ints[8]);
      
      if (debug_)
      {
        ROS_INFO("Welding State: arc_ok=%s, power_err=%s, voltage=%.1fV, current=%.0fA",
                 weld_msg.arc_ok ? "TRUE" : "FALSE",
                 weld_msg.power_err ? "TRUE" : "FALSE",
                 weld_msg.act_voltage * VOLTAGE_SCALE,
                 weld_msg.act_current * CURRENT_SCALE);
      }
      
      // Publish message
      weld_state_pub_.publish(weld_msg);
      
      // Reset error count on success
      error_count = 0;
      
      ros::spinOnce();
    }
  }
  
private:
  bool receiveExactly(uint8_t* buffer, size_t bytes)
  {
    size_t received = 0;
    while (received < bytes)
    {
      ssize_t result = recv(sock_fd_, buffer + received, bytes - received, 0);
      if (result <= 0)
      {
        if (result == 0)
        {
          ROS_WARN("Connection closed by server");
        }
        else
        {
          ROS_WARN("recv() error: %s", strerror(errno));
        }
        return false;
      }
      received += static_cast<size_t>(result);
    }
    return true;
  }
  
  bool reconnect()
  {
    ROS_INFO("Attempting to reconnect...");
    
    if (sock_fd_ >= 0)
    {
      close(sock_fd_);
      sock_fd_ = -1;
    }
    
    sleep(1); // Wait before reconnecting
    
    return init();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fanuc_weld_state_node_tcp");
  
  FanucWeldStateNodeTcp node;
  
  if (!node.init())
  {
    ROS_ERROR("Failed to initialize Fanuc Weld State TCP Node");
    return -1;
  }
  
  ROS_INFO("Fanuc Weld State TCP Node initialized successfully");
  node.run();
  
  return 0;
} 
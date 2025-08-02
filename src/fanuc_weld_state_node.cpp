#include <ros/ros.h>
#include <fanuc_driver/WeldState.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>
#include <cctype>

// Scaling factors from EWM manual
const double VOLTAGE_SCALE = 100.0 / 32767.0;   // Raw to Volts
const double CURRENT_SCALE = 1000.0 / 32767.0;  // Raw to Amperes
//const double WIRE_SPEED_SCALE = 40.0 / 32767.0; // Raw to m/min

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
  
public:
  FanucWeldStateNodeTcp() : nh_("~"), sock_fd_(-1)
  {
    // Get parameters
    nh_.param<std::string>("robot_ip", robot_ip_, "127.0.0.1");
    nh_.param<int>("robot_port", robot_port_, 11002);
    nh_.param<bool>("debug", debug_, false);

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
      
      // Validate header - expecting Standard Simple Message format with sequence number
      if (length != 49 || msg_type != 15 || comm_type != 1)
      {
        ROS_WARN("Invalid header format - Expected: Length=49, Type=15, Comm=1");
        if (debug_)
        {
          ROS_WARN("Received: Length=%u, Type=%u, Comm=%u, Reply=%u", length, msg_type, comm_type, reply_type);
        }
        continue;
      }
      
      // Receive sequence number (4 bytes)
      uint8_t seq_buf[4];
      if (!receiveExactly(seq_buf, 4))
      {
        ROS_ERROR("Failed to receive sequence number");
        continue;
      }
      uint32_t seq_nr = readUint32(&seq_buf[0]);
      
      // Receive payload (32 bytes of weld data)
      uint8_t payload_buf[32];
      if (!receiveExactly(payload_buf, 32))
      {
        ROS_ERROR("Failed to receive payload");
        continue;
      }
      
      // Parse weld data (Little Endian)
      int32_t weld_ints[8];
      for (int i = 0; i < 8; i++)
      {
        weld_ints[i] = readInt32(&payload_buf[i * 4]);
      }
      
      if (debug_)
      {
        ROS_INFO("Seq: %u, Raw weld data:", seq_nr);
        ROS_INFO("  arc_ok: %d, ready: %d, stick_err: %d, power_err: %d", 
                 weld_ints[0], weld_ints[1], weld_ints[2], weld_ints[3]);
        ROS_INFO("  depos_di: %d, voltage: %d, current: %d, wire_spd: %d",
                 weld_ints[4], weld_ints[5], weld_ints[6], weld_ints[7]);
      }
      
      // Create ROS message
      fanuc_driver::WeldState weld_msg;
      weld_msg.arc_ok = (weld_ints[0] != 0);
      weld_msg.ready = (weld_ints[1] != 0);
      weld_msg.stick_err = (weld_ints[2] != 0);
      weld_msg.power_err = (weld_ints[3] != 0);
      weld_msg.depos_di = (weld_ints[4] != 0);
      weld_msg.act_voltage = static_cast<int16_t>(weld_ints[5]);
      weld_msg.act_current = static_cast<int16_t>(weld_ints[6]);
      weld_msg.act_wire_spd = static_cast<int16_t>(weld_ints[7]);
      
      if (debug_)
      {
        ROS_INFO("Welding State: arc_ok=%s, ready=%s, voltage=%.1fV, current=%.0fA",
                 weld_msg.arc_ok ? "TRUE" : "FALSE",
                 weld_msg.ready ? "TRUE" : "FALSE",
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
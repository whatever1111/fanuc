#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "fanuc_driver/msg/weld_command.hpp"
#include "std_msgs/msg/header.hpp"
// Optional: parse unified IO map file for robot ip/port/byte_order
#include <yaml-cpp/yaml.h>

class FanucWeldCommandNode : public rclcpp::Node {
public:
	FanucWeldCommandNode()
		: rclcpp::Node("fanuc_weld_command_node"),
		  sock_fd_(-1),
		  connected_(false),
		  little_endian_(true),
		  heartbeat_enabled_(false),
		  sequence_number_(0) {
		// Parameters
		robot_ip_ = this->declare_parameter<std::string>("robot_ip", "127.0.0.1");
		robot_port_ = this->declare_parameter<int>("robot_port", 11002);
		// Optional unified-template override for command port
		{
			int port_override = this->declare_parameter<int>("port_weld_command", 0);
			if (port_override > 0) {
				robot_port_ = port_override;
			}
		}
		std::string byte_order_param = this->declare_parameter<std::string>("byte_order", "little");
		std::transform(byte_order_param.begin(), byte_order_param.end(), byte_order_param.begin(), ::tolower);
		little_endian_ = (byte_order_param == "little" || byte_order_param == "le");

		// Optional: unified io_map YAML file to override ip/port/byte_order
		{
			std::string io_map_file = this->declare_parameter<std::string>("io_map_file", "");
			if (!io_map_file.empty()) {
				try {
					YAML::Node root = YAML::LoadFile(io_map_file);
					if (root["robot"]) {
						YAML::Node robot = root["robot"];
						if (robot["ip"]) {
							robot_ip_ = robot["ip"].as<std::string>();
						}
						if (robot["byte_order"]) {
							std::string bo = robot["byte_order"].as<std::string>();
							std::transform(bo.begin(), bo.end(), bo.begin(), ::tolower);
							little_endian_ = (bo == "little" || bo == "le");
						}
						if (robot["ports"] && robot["ports"]["weld_command_server"]) {
							int port_from_yaml = robot["ports"]["weld_command_server"].as<int>();
							if (port_from_yaml > 0) {
								robot_port_ = port_from_yaml;
							}
						}
					}
					RCLCPP_INFO(this->get_logger(), "Using io_map_file overrides: %s:%d (%s)",
						robot_ip_.c_str(), robot_port_, little_endian_ ? "little" : "big");
				} catch (const std::exception & ex) {
					RCLCPP_WARN(this->get_logger(), "Failed to parse io_map_file '%s': %s", io_map_file.c_str(), ex.what());
				}
			}
		}
		RCLCPP_INFO(this->get_logger(), "Expecting %s-endian robot messages", little_endian_ ? "little" : "big");

		heartbeat_enabled_ = this->declare_parameter<bool>("enable_heartbeat", false);
		if (heartbeat_enabled_) {
			heartbeat_timer_ = this->create_wall_timer(
				std::chrono::seconds(3),
				std::bind(&FanucWeldCommandNode::heartbeatCallback, this));
			RCLCPP_INFO(this->get_logger(), "Heartbeat enabled (3s)");
		} else {
			RCLCPP_INFO(this->get_logger(), "Heartbeat disabled");
		}

		RCLCPP_INFO(this->get_logger(), "Target robot: %s:%d", robot_ip_.c_str(), robot_port_);

		command_sub_ = this->create_subscription<fanuc_driver::msg::WeldCommand>(
			"/weld_command", 10,
			std::bind(&FanucWeldCommandNode::commandCallback, this, std::placeholders::_1));
	}

	~FanucWeldCommandNode() override {
		std::lock_guard<std::mutex> lk(socket_mutex_);
		disconnect();
	}

private:
	struct WeldCommandPacket {
		uint32_t length;
		uint32_t msg_type;
		uint32_t comm_type;
		uint32_t reply_type;
		uint32_t seq_nr;
		uint32_t target_wire_spd;
		uint32_t correction_val;
		uint32_t dyn_setting;
		uint32_t operation_mode;
		uint32_t std_pulse_val;
		uint32_t program_number;
		uint32_t arc_start_cmd;
		uint32_t gas_control;
		uint32_t jog_feed_cmd;
		uint32_t jog_retract_cmd;
	} __attribute__((packed));

	static constexpr uint32_t RI_MT_WELDCMD = 14;  // Weld Command message type
	static constexpr uint32_t RI_CT_SVCREQ = 1;   // Service Request
	static constexpr uint32_t RI_RT_INVAL = 0;    // Invalid reply type for requests
	static constexpr uint32_t NO_CHANGE = 0xFFFFFFFF;  // Sentinel for unchanged fields
	static constexpr int32_t RI_SEQ_HB = -110;    // Heartbeat sequence number

	bool connect() {
		if (connected_) return true;

		sock_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
		if (sock_fd_ < 0) {
			RCLCPP_ERROR(this->get_logger(), "Failed to create socket: %s", std::strerror(errno));
			return false;
		}

		sockaddr_in server_addr{};
		server_addr.sin_family = AF_INET;
		server_addr.sin_port = htons(static_cast<uint16_t>(robot_port_));
		if (::inet_pton(AF_INET, robot_ip_.c_str(), &server_addr.sin_addr) <= 0) {
			RCLCPP_ERROR(this->get_logger(), "Invalid IP address: %s", robot_ip_.c_str());
			::close(sock_fd_);
			sock_fd_ = -1;
			return false;
		}

		if (::connect(sock_fd_, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
			RCLCPP_WARN(this->get_logger(), "Failed to connect to robot at %s:%d - %s", robot_ip_.c_str(), robot_port_, std::strerror(errno));
			::close(sock_fd_);
			sock_fd_ = -1;
			return false;
		}
		connected_ = true;
		RCLCPP_INFO(this->get_logger(), "Connected to robot at %s:%d", robot_ip_.c_str(), robot_port_);
		return true;
	}

	void disconnect() {
		if (sock_fd_ >= 0) {
			::shutdown(sock_fd_, SHUT_RDWR);
			::close(sock_fd_);
			sock_fd_ = -1;
		}
		connected_ = false;
	}

	void commandCallback(const fanuc_driver::msg::WeldCommand & msg) {
		RCLCPP_INFO(this->get_logger(), "--- Received WeldCommand ROS message ---");
		RCLCPP_INFO(this->get_logger(), "  prog_number: %d", static_cast<int32_t>(msg.program_number));
		RCLCPP_INFO(this->get_logger(), "  target_wire_spd: %d", static_cast<int32_t>(msg.target_wire_spd));
		RCLCPP_INFO(this->get_logger(), "  correction_val: %d", static_cast<int32_t>(msg.correction_val));
		RCLCPP_INFO(this->get_logger(), "  dyn_setting: %d", static_cast<int32_t>(msg.dyn_setting));
		RCLCPP_INFO(this->get_logger(), "  operation_mode: %d", static_cast<int32_t>(msg.operation_mode));
		RCLCPP_INFO(this->get_logger(), "  std_pulse_val: %d", static_cast<int32_t>(msg.std_pulse_val));
		RCLCPP_INFO(this->get_logger(), "  arc_start_cmd: %d", static_cast<int32_t>(msg.arc_start_cmd));
		RCLCPP_INFO(this->get_logger(), "  gas_control: %d", static_cast<int32_t>(msg.gas_control));
		RCLCPP_INFO(this->get_logger(), "  jog_feed_cmd: %d", static_cast<int32_t>(msg.jog_feed_cmd));
		RCLCPP_INFO(this->get_logger(), "  jog_retract_cmd: %d", static_cast<int32_t>(msg.jog_retract_cmd));

		std::lock_guard<std::mutex> lk(socket_mutex_);
		if (!connect()) {
			RCLCPP_WARN(this->get_logger(), "Cannot send command - not connected to robot");
			return;
		}

		WeldCommandPacket packet{};
		packet.length = sizeof(packet);
		packet.msg_type = RI_MT_WELDCMD;
		packet.comm_type = RI_CT_SVCREQ;
		packet.reply_type = RI_RT_INVAL;
		packet.seq_nr = ++sequence_number_;

		packet.target_wire_spd = static_cast<uint32_t>(msg.target_wire_spd);
		packet.correction_val  = static_cast<uint32_t>(msg.correction_val);
		packet.dyn_setting     = static_cast<uint32_t>(msg.dyn_setting);
		packet.operation_mode  = static_cast<uint32_t>(msg.operation_mode);
		packet.std_pulse_val   = static_cast<uint32_t>(msg.std_pulse_val);
		packet.program_number  = static_cast<uint32_t>(msg.program_number);
		packet.arc_start_cmd   = static_cast<uint32_t>(msg.arc_start_cmd);
		packet.gas_control     = static_cast<uint32_t>(msg.gas_control);
		packet.jog_feed_cmd    = static_cast<uint32_t>(msg.jog_feed_cmd);
		packet.jog_retract_cmd = static_cast<uint32_t>(msg.jog_retract_cmd);

		uint8_t send_buf[sizeof(packet)];
		std::memcpy(send_buf, &packet, sizeof(packet));
		if (!little_endian_) {
			uint32_t *p = reinterpret_cast<uint32_t*>(send_buf);
			size_t cnt = sizeof(packet) / sizeof(uint32_t);
			for (size_t i = 0; i < cnt; ++i) {
				p[i] = htonl(p[i]);
			}
		}

		RCLCPP_INFO(this->get_logger(), "--- Sending TCP Packet (seq: %u) ---", packet.seq_nr);
		RCLCPP_INFO(this->get_logger(), "  prog_number: %d", static_cast<int32_t>(packet.program_number));
		RCLCPP_INFO(this->get_logger(), "  target_wire_spd: %d", static_cast<int32_t>(packet.target_wire_spd));
		RCLCPP_INFO(this->get_logger(), "  correction_val: %d", static_cast<int32_t>(packet.correction_val));
		RCLCPP_INFO(this->get_logger(), "  dyn_setting: %d", static_cast<int32_t>(packet.dyn_setting));
		RCLCPP_INFO(this->get_logger(), "  operation_mode: %d", static_cast<int32_t>(packet.operation_mode));
		RCLCPP_INFO(this->get_logger(), "  std_pulse_val: %d", static_cast<int32_t>(packet.std_pulse_val));
		RCLCPP_INFO(this->get_logger(), "  arc_start_cmd: %d", static_cast<int32_t>(packet.arc_start_cmd));
		RCLCPP_INFO(this->get_logger(), "  gas_control: %d", static_cast<int32_t>(packet.gas_control));
		RCLCPP_INFO(this->get_logger(), "  jog_feed_cmd: %d", static_cast<int32_t>(packet.jog_feed_cmd));
		RCLCPP_INFO(this->get_logger(), "  jog_retract_cmd: %d", static_cast<int32_t>(packet.jog_retract_cmd));

		ssize_t bytes_sent = ::send(sock_fd_, send_buf, sizeof(packet), 0);
		if (bytes_sent != static_cast<ssize_t>(sizeof(packet))) {
			RCLCPP_ERROR(this->get_logger(), "Failed to send complete packet: sent %zd of %zu bytes", bytes_sent, sizeof(packet));
			disconnect();
			return;
		}

		struct AckReply { uint32_t length, msg_type, comm_type, reply_type, seq_nr; } reply{};
		size_t expected_bytes = sizeof(reply);
		size_t received_total = 0;
		while (received_total < expected_bytes) {
			ssize_t r = ::recv(sock_fd_, reinterpret_cast<char*>(&reply) + received_total,
							expected_bytes - received_total, MSG_DONTWAIT);
			if (r < 0) {
				if (errno == EAGAIN || errno == EWOULDBLOCK) break;
				RCLCPP_WARN(this->get_logger(), "recv() error while waiting for ACK: %s", std::strerror(errno));
				disconnect();
				break;
			} else if (r == 0) {
				RCLCPP_WARN(this->get_logger(), "Connection closed by robot while waiting for ACK");
				disconnect();
				break;
			} else {
				received_total += static_cast<size_t>(r);
			}
		}

		if (!little_endian_) {
			reply.length     = ntohl(reply.length);
			reply.msg_type   = ntohl(reply.msg_type);
			reply.comm_type  = ntohl(reply.comm_type);
			reply.reply_type = ntohl(reply.reply_type);
			reply.seq_nr     = ntohl(reply.seq_nr);
		}

		if (received_total == expected_bytes) {
			if (reply.reply_type == 1) {
				RCLCPP_DEBUG(this->get_logger(), "Command acknowledged successfully (seq: %u)", reply.seq_nr);
			} else {
				RCLCPP_WARN(this->get_logger(), "Command failed on robot (seq: %u, reply_type: %u)", reply.seq_nr, reply.reply_type);
			}
		} else {
			RCLCPP_WARN(this->get_logger(), "Incomplete acknowledgment received: %zu of %zu bytes", received_total, expected_bytes);
		}
	}

	void heartbeatCallback() {
		std::lock_guard<std::mutex> lk(socket_mutex_);
		if (!connected_) return;

		WeldCommandPacket packet{};
		packet.length = sizeof(packet);
		packet.msg_type = RI_MT_WELDCMD;
		packet.comm_type = RI_CT_SVCREQ;
		packet.reply_type = RI_RT_INVAL;
		packet.seq_nr = static_cast<uint32_t>(RI_SEQ_HB);
		packet.target_wire_spd = NO_CHANGE;
		packet.correction_val = NO_CHANGE;
		packet.dyn_setting = NO_CHANGE;
		packet.operation_mode = NO_CHANGE;
		packet.std_pulse_val = NO_CHANGE;
		packet.program_number = NO_CHANGE;
		packet.arc_start_cmd = NO_CHANGE;
		packet.gas_control = NO_CHANGE;
		packet.jog_feed_cmd = NO_CHANGE;
		packet.jog_retract_cmd = NO_CHANGE;

		uint8_t send_buf[sizeof(packet)];
		std::memcpy(send_buf, &packet, sizeof(packet));
		if (!little_endian_) {
			uint32_t *p = reinterpret_cast<uint32_t*>(send_buf);
			size_t cnt = sizeof(packet) / sizeof(uint32_t);
			for (size_t i = 0; i < cnt; ++i) { p[i] = htonl(p[i]); }
		}

		ssize_t bytes_sent = ::send(sock_fd_, send_buf, sizeof(packet), 0);
		if (bytes_sent != static_cast<ssize_t>(sizeof(packet))) {
			RCLCPP_WARN(this->get_logger(), "Failed to send heartbeat packet, disconnecting");
			disconnect();
			return;
		}

		struct AckReply { uint32_t length, msg_type, comm_type, reply_type, seq_nr; } hb_reply{};
		size_t expected_bytes = sizeof(hb_reply);
		size_t received_total = 0;
		while (received_total < expected_bytes) {
			ssize_t r = ::recv(sock_fd_, reinterpret_cast<char*>(&hb_reply) + received_total,
							expected_bytes - received_total, MSG_DONTWAIT);
			if (r <= 0) break;
			received_total += static_cast<size_t>(r);
		}
	}

	// Fields
	std::string robot_ip_;
	int robot_port_;
	int sock_fd_;
	bool connected_;
	bool little_endian_;
	bool heartbeat_enabled_;
	std::mutex socket_mutex_;
	uint32_t sequence_number_;

	rclcpp::Subscription<fanuc_driver::msg::WeldCommand>::SharedPtr command_sub_;
	rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<FanucWeldCommandNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

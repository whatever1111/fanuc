/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, robopath
 * All rights reserved.
 *
 * This node connects to a Fanuc KAREL TCP service that streams weld state
 * data using a fixed-length binary frame (header + payload) and publishes
 * `/weld_state` messages. It supports both little-endian and big-endian
 * payloads as configured by the `byte_order` parameter. The implementation
 * accepts an optional trailing CR/LF after frames and attempts lightweight
 * reconnection when communication errors occur.
 *
 * Parameters: robot_ip (string), robot_port (int), byte_order (string),
 * debug (bool), dump_raw (bool), expected_msg_type (int), expected_comm_type
 * (int), payload_fields (int). See `config/weld_state.yaml`.
 */

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "fanuc_driver/msg/weld_state.hpp"
// Optional: parse unified IO map file for robot ip/port/byte_order
#include <yaml-cpp/yaml.h>

class FanucWeldStateNodeTcp : public rclcpp::Node {
public:
	FanucWeldStateNodeTcp()
		: rclcpp::Node("fanuc_weld_state_node"),
		  sock_fd_(-1),
		  connected_(false),
		  little_endian_(true),
		  debug_(false),
		  dump_raw_(false),
		  expected_standard_length_(53),
		  expected_msg_type_(15),
		  expected_comm_type_(1),
		  payload_fields_(9),
		  payload_size_bytes_(static_cast<std::size_t>(payload_fields_) * sizeof(int32_t)),
		  stop_thread_(false) {
		// Parameters
		robot_ip_ = this->declare_parameter<std::string>("robot_ip", "127.0.0.1");
		robot_port_ = this->declare_parameter<int>("robot_port", 11002);
		// Optional unified-template override for state port
		{
			int port_override = this->declare_parameter<int>("port_weld_state", 0);
			if (port_override > 0) {
				robot_port_ = port_override;
			}
		}
		debug_ = this->declare_parameter<bool>("debug", false);
		dump_raw_ = this->declare_parameter<bool>("dump_raw", false);
		int tmp_int = 0;
		tmp_int = this->declare_parameter<int>("expected_standard_length", 53);
		expected_standard_length_ = static_cast<uint32_t>(tmp_int);
		tmp_int = this->declare_parameter<int>("expected_msg_type", 15);
		expected_msg_type_ = static_cast<uint32_t>(tmp_int);
		tmp_int = this->declare_parameter<int>("expected_comm_type", 1);
		expected_comm_type_ = static_cast<uint32_t>(tmp_int);
		payload_fields_ = this->declare_parameter<int>("payload_fields", 9);
		payload_size_bytes_ = static_cast<std::size_t>(payload_fields_) * sizeof(int32_t);

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
						if (robot["ports"] && robot["ports"]["weld_state_server"]) {
							int port_from_yaml = robot["ports"]["weld_state_server"].as<int>();
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

		RCLCPP_INFO(this->get_logger(), "Fanuc Weld State TCP Node starting");
		RCLCPP_INFO(this->get_logger(), "Target robot: %s:%d", robot_ip_.c_str(), robot_port_);
		RCLCPP_INFO(this->get_logger(), "Expecting %s-endian robot messages", little_endian_ ? "little" : "big");
		if (debug_) RCLCPP_INFO(this->get_logger(), "Debug mode enabled");

		weld_state_pub_ = this->create_publisher<fanuc_driver::msg::WeldState>("/weld_state", 10);

		recv_thread_ = std::thread(&FanucWeldStateNodeTcp::receiveLoop, this);
	}

	~FanucWeldStateNodeTcp() override {
		stop_thread_.store(true);
		if (recv_thread_.joinable()) {
			recv_thread_.join();
		}
		std::lock_guard<std::mutex> lk(socket_mutex_);
		disconnect();
	}

private:
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

	bool receiveExactly(uint8_t * buffer, std::size_t bytes) {
		std::size_t received = 0;
		while (received < bytes) {
			ssize_t result = ::recv(sock_fd_, buffer + received, bytes - received, 0);
			if (result <= 0) {
				if (result == 0) {
					RCLCPP_WARN(this->get_logger(), "Connection closed by server");
				} else {
					RCLCPP_WARN(this->get_logger(), "recv() error: %s", std::strerror(errno));
				}
				return false;
			}
			received += static_cast<std::size_t>(result);
		}
		return true;
	}

	void receiveLoop() {
		int error_count = 0;
		int msg_count = 0;

		while (rclcpp::ok() && !stop_thread_.load()) {
			{
				std::lock_guard<std::mutex> lk(socket_mutex_);
				if (!connect()) {
					// Retry after short delay
				}
			}
			if (!connected_) {
				std::this_thread::sleep_for(std::chrono::milliseconds(500));
				continue;
			}

			uint8_t header_buf[16];
			if (!receiveExactly(header_buf, 16)) {
				error_count++;
				if (error_count % 10 == 1) {
					RCLCPP_WARN(this->get_logger(), "Communication error (count: %d)", error_count);
				}
				if (error_count > 5) {
					std::lock_guard<std::mutex> lk(socket_mutex_);
					disconnect();
				}
				continue;
			}

			auto readUint32 = [this](const uint8_t * data) -> uint32_t {
				uint32_t val;
				std::memcpy(&val, data, sizeof(uint32_t));
				if (little_endian_) return val; // host is little-endian
				return ntohl(val);
			};

			auto readInt32 = [this](const uint8_t * data) -> int32_t {
				uint32_t tmp;
				std::memcpy(&tmp, data, sizeof(uint32_t));
				if (!little_endian_) tmp = ntohl(tmp);
				return static_cast<int32_t>(tmp);
			};

			uint32_t length     = readUint32(&header_buf[0]);
			uint32_t msg_type   = readUint32(&header_buf[4]);
			uint32_t comm_type  = readUint32(&header_buf[8]);
			uint32_t reply_type = readUint32(&header_buf[12]);

			msg_count++;
			if (debug_) {
				RCLCPP_INFO(this->get_logger(), "Message #%d - Length: %u, Type: %u, Comm: %u, Reply: %u",
					msg_count, length, msg_type, comm_type, reply_type);
			}

			if (msg_type != expected_msg_type_ || comm_type != expected_comm_type_ ||
				length < (4 + payload_size_bytes_)) {
				RCLCPP_WARN(this->get_logger(), "Invalid header - Expect: msg_type=%u, comm_type=%u, length>=%zu",
					expected_msg_type_, expected_comm_type_, 4 + payload_size_bytes_);
				if (debug_) {
					RCLCPP_WARN(this->get_logger(), "Received: Length=%u, Type=%u, Comm=%u, Reply=%u",
						length, msg_type, comm_type, reply_type);
				}
				if (length > 0) {
					std::vector<uint8_t> junk(length);
					receiveExactly(junk.data(), length);
				}
				continue;
			}

			if (length < 12) {
				RCLCPP_WARN(this->get_logger(), "Length field too small (%u) – skipping", length);
				continue;
			}
			uint32_t remaining_bytes = length - 12;
			std::vector<uint8_t> remainder(remaining_bytes);
			if (!receiveExactly(remainder.data(), remaining_bytes)) {
				RCLCPP_ERROR(this->get_logger(), "Failed to receive remaining %u bytes", remaining_bytes);
				continue;
			}

			std::size_t trim_bytes = 0;
			while (trim_bytes < 2 && !remainder.empty()) {
				uint8_t last = remainder[remainder.size() - 1 - trim_bytes];
				if (last == '\r' || last == '\n') ++trim_bytes; else break;
			}
			std::size_t data_bytes = remaining_bytes - trim_bytes;

			std::size_t payload_offset = 0;
			uint32_t seq_nr = 0;

			if (data_bytes == payload_size_bytes_ || data_bytes == payload_size_bytes_ + 1) {
				payload_offset = 0;
			} else if (data_bytes == payload_size_bytes_ + 4 || data_bytes == payload_size_bytes_ + 5) {
				seq_nr = readUint32(&remainder[0]);
				payload_offset = 4;
			} else {
				RCLCPP_WARN(this->get_logger(), "Unexpected data bytes %zu (accept %zu/%zu/%zu/%zu) – skipping",
					data_bytes, payload_size_bytes_, payload_size_bytes_ + 1,
					payload_size_bytes_ + 4, payload_size_bytes_ + 5);
				continue;
			}

			const uint8_t * payload_ptr = &remainder[payload_offset];
			std::size_t available_payload_bytes = data_bytes - payload_offset;
			std::size_t ints_in_msg = available_payload_bytes / 4;
			if (ints_in_msg < static_cast<std::size_t>(payload_fields_)) {
				RCLCPP_WARN(this->get_logger(), "Payload too small (%zu ints) – skipping", ints_in_msg);
				continue;
			}

			std::vector<int32_t> weld_ints(payload_fields_);
			for (int i = 0; i < payload_fields_; ++i) {
				weld_ints[i] = readInt32(payload_ptr + i * 4);
			}

			// consume any leftover CR/LF still in socket (not counted in length)
			while (true) {
				uint8_t peek;
				ssize_t r = ::recv(sock_fd_, &peek, 1, MSG_PEEK | MSG_DONTWAIT);
				if (r == 1 && (peek == '\r' || peek == '\n')) {
					::recv(sock_fd_, &peek, 1, 0);
				} else {
					break;
				}
			}

			if (debug_) {
				RCLCPP_INFO(this->get_logger(), "Seq: %u, Raw weld data:", seq_nr);
				RCLCPP_INFO(this->get_logger(), "  arc_ok: %d, power_err: %d, depos_di: %d",
					weld_ints[0], weld_ints[1], weld_ints[2]);
				RCLCPP_INFO(this->get_logger(), "  ewm_err: %d, warn: %d, voltage: %d, current: %d, wire_spd: %d, motor_curr: %d",
					weld_ints[3], weld_ints[4], weld_ints[5], weld_ints[6], weld_ints[7], weld_ints[8]);
			}

			fanuc_driver::msg::WeldState weld_msg;
			weld_msg.arc_ok = (weld_ints[0] != 0);
			weld_msg.power_err = (weld_ints[1] != 0);
			weld_msg.depos_di = (weld_ints[2] != 0);
			weld_msg.ewm_err = static_cast<int16_t>(weld_ints[3]);
			weld_msg.warning_state = static_cast<int16_t>(weld_ints[4]);
			weld_msg.act_voltage = static_cast<int16_t>(weld_ints[5]);
			weld_msg.act_current = static_cast<int16_t>(weld_ints[6]);
			weld_msg.act_wire_spd = static_cast<int16_t>(weld_ints[7]);
			weld_msg.motor_current = static_cast<int16_t>(weld_ints[8]);

			weld_state_pub_->publish(weld_msg);
			error_count = 0;
		}
	}

	// Fields
	std::string robot_ip_;
	int robot_port_;
	int sock_fd_;
	bool connected_;
	bool little_endian_;
	bool debug_;
	bool dump_raw_;
	uint32_t expected_standard_length_;
	uint32_t expected_msg_type_;
	uint32_t expected_comm_type_;
	int payload_fields_;
	std::size_t payload_size_bytes_;
	std::mutex socket_mutex_;
	std::thread recv_thread_;
	std::atomic<bool> stop_thread_;

	rclcpp::Publisher<fanuc_driver::msg::WeldState>::SharedPtr weld_state_pub_;
};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<FanucWeldStateNodeTcp>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}


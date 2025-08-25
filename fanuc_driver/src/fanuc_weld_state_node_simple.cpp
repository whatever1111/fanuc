/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, robopath
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Fanuc Weld State Node (SimpleMessage variant)
 *
 * Purpose
 * - Connects to a Fanuc controller KAREL program that publishes weld state
 *   over a SimpleMessage-like TCP stream.
 * - Parses frames in one of two formats and publishes `fanuc_driver/msg/WeldState`:
 *   1) Payload-only format:       length=32, then 9×int32 payload
 *   2) Standard format with seq:  length=53, then 4B seq + 9×int32 payload
 * - Optional single CR/LF terminator is tolerated after payload.
 *
 * Parameters (ROS 2, declared at startup)
 * - robot_ip (string, default "127.0.0.1")
 * - robot_port (int, default 11002)
 * - byte_order (string, default "little"): "little" or "big"
 * - debug (bool, default false)
 * - payload_only_length (int, default 32)
 * - standard_format_length (int, default 53)
 * - expected_msg_type (int, default 15)
 * - expected_comm_type (int, default 1)
 * - num_weld_fields (int, default 9)
 *
 * Topic
 * - Publishes `/weld_state` (fanuc_driver/msg/WeldState)
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
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "fanuc_driver/msg/weld_state.hpp"
// Optional: parse unified IO map file for robot ip/port/byte_order
#include <yaml-cpp/yaml.h>

class FanucWeldStateNodeSimple : public rclcpp::Node {
public:
	FanucWeldStateNodeSimple()
		: rclcpp::Node("fanuc_weld_state_node_simple"),
		  sock_fd_(-1),
		  connected_(false),
		  little_endian_(true),
		  debug_(false),
		  payload_only_length_(32),
		  standard_format_length_(53),
		  expected_msg_type_(15),
		  expected_comm_type_(1),
		  num_weld_fields_(9),
		  payload_size_bytes_(static_cast<std::size_t>(num_weld_fields_) * sizeof(int32_t)),
		  stop_thread_(false) {
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
		int tmp = 0;
		tmp = this->declare_parameter<int>("payload_only_length", 32); payload_only_length_ = static_cast<uint32_t>(tmp);
		tmp = this->declare_parameter<int>("standard_format_length", 53); standard_format_length_ = static_cast<uint32_t>(tmp);
		tmp = this->declare_parameter<int>("expected_msg_type", 15); expected_msg_type_ = static_cast<uint32_t>(tmp);
		tmp = this->declare_parameter<int>("expected_comm_type", 1); expected_comm_type_ = static_cast<uint32_t>(tmp);
		num_weld_fields_ = this->declare_parameter<int>("num_weld_fields", 9);
		payload_size_bytes_ = static_cast<std::size_t>(num_weld_fields_) * sizeof(int32_t);

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

		RCLCPP_INFO(this->get_logger(), "Fanuc Weld State SimpleMessage Node starting");
		RCLCPP_INFO(this->get_logger(), "Target robot: %s:%d", robot_ip_.c_str(), robot_port_);
		RCLCPP_INFO(this->get_logger(), "Expecting %s-endian robot messages", little_endian_ ? "little" : "big");
		if (debug_) RCLCPP_INFO(this->get_logger(), "Debug mode enabled");

		weld_state_pub_ = this->create_publisher<fanuc_driver::msg::WeldState>("/weld_state", 10);
		recv_thread_ = std::thread(&FanucWeldStateNodeSimple::receiveLoop, this);
	}

	~FanucWeldStateNodeSimple() override {
		stop_thread_.store(true);
		if (recv_thread_.joinable()) recv_thread_.join();
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
				if (!connect()) {}
			}
			if (!connected_) { std::this_thread::sleep_for(std::chrono::milliseconds(500)); continue; }

			uint8_t header_buf[16];
			if (!receiveExactly(header_buf, 16)) { error_count++; if (error_count > 5) { std::lock_guard<std::mutex> lk(socket_mutex_); disconnect(); } continue; }

			auto readUint32 = [this](const uint8_t * data) -> uint32_t { uint32_t v; std::memcpy(&v, data, sizeof(uint32_t)); return little_endian_ ? v : ntohl(v); };
			auto readInt32  = [this](const uint8_t * data) -> int32_t  { uint32_t t; std::memcpy(&t, data, sizeof(uint32_t)); if (!little_endian_) t = ntohl(t); return static_cast<int32_t>(t); };

			uint32_t length     = readUint32(&header_buf[0]);
			uint32_t msg_type   = readUint32(&header_buf[4]);
			uint32_t comm_type  = readUint32(&header_buf[8]);
			uint32_t reply_type = readUint32(&header_buf[12]);

			msg_count++;
			if (debug_) {
				RCLCPP_INFO(this->get_logger(), "Message #%d - Length: %u, Type: %u, Comm: %u, Reply: %u",
					msg_count, length, msg_type, comm_type, reply_type);
			}

			std::vector<int32_t> weld_ints;
			if (length == payload_only_length_ && msg_type == expected_msg_type_ && comm_type == expected_comm_type_) {
				std::vector<uint8_t> buf(payload_only_length_);
				if (!receiveExactly(buf.data(), buf.size())) { continue; }
				weld_ints.resize(num_weld_fields_);
				for (std::size_t i = 0; i < num_weld_fields_; ++i) weld_ints[i] = readInt32(buf.data() + i * 4);
				uint8_t peek; ssize_t r = ::recv(sock_fd_, &peek, 1, MSG_PEEK | MSG_DONTWAIT); if (r == 1 && (peek == '\r' || peek == '\n')) { ::recv(sock_fd_, &peek, 1, 0); }
			} else if (length == standard_format_length_ && msg_type == expected_msg_type_ && comm_type == expected_comm_type_) {
				std::vector<uint8_t> buf(4 + payload_size_bytes_);
				if (!receiveExactly(buf.data(), buf.size())) { continue; }
				const uint8_t * payload = buf.data() + 4; // skip seq
				weld_ints.resize(num_weld_fields_);
				for (std::size_t i = 0; i < num_weld_fields_; ++i) weld_ints[i] = readInt32(payload + i * 4);
				uint8_t peek; ssize_t r = ::recv(sock_fd_, &peek, 1, MSG_PEEK | MSG_DONTWAIT); if (r == 1 && (peek == '\r' || peek == '\n')) { ::recv(sock_fd_, &peek, 1, 0); }
			} else {
				RCLCPP_WARN(this->get_logger(), "Invalid header - Expected: Length=32 or 53, Type=%u, Comm=%u", expected_msg_type_, expected_comm_type_);
				continue;
			}

			if (weld_ints.size() < static_cast<std::size_t>(num_weld_fields_)) { continue; }

			fanuc_driver::msg::WeldState weld_msg;
			weld_msg.arc_ok        = (weld_ints[0] != 0);
			weld_msg.power_err     = (weld_ints[1] != 0);
			weld_msg.depos_di      = (weld_ints[2] != 0);
			weld_msg.ewm_err       = static_cast<int16_t>(weld_ints[3]);
			weld_msg.warning_state = static_cast<int16_t>(weld_ints[4]);
			weld_msg.act_voltage   = static_cast<int16_t>(weld_ints[5]);
			weld_msg.act_current   = static_cast<int16_t>(weld_ints[6]);
			weld_msg.act_wire_spd  = static_cast<int16_t>(weld_ints[7]);
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
	uint32_t payload_only_length_;
	uint32_t standard_format_length_;
	uint32_t expected_msg_type_;
	uint32_t expected_comm_type_;
	int num_weld_fields_;
	std::size_t payload_size_bytes_;
	std::mutex socket_mutex_;
	std::thread recv_thread_;
	std::atomic<bool> stop_thread_;

	rclcpp::Publisher<fanuc_driver::msg::WeldState>::SharedPtr weld_state_pub_;
};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<FanucWeldStateNodeSimple>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}


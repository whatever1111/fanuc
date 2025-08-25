#!/usr/bin/env python3
"""
Integration test for `weld_state_node_tcp`.

This test launches the ROS 2 node with a loopback TCP mock that emits
well-formed weld state frames. It subscribes to `/weld_state` and validates
that the parsed message fields match the payload data.
"""
import socket
import struct
import threading
import time

import pytest

import rclpy
from rclpy.node import Node
import launch
import launch_ros
import launch_testing
from fanuc_driver.msg import WeldState


class TcpStateServer:
    """TCP server that streams weld state frames to one client.

    If `port` is 0, an ephemeral port is chosen and exposed via `self.port`.
    Frames comply with the node's header checks and carry 9×int32 payload.
    """

    def __init__(self, host: str = '127.0.0.1', port: int = 0, endian: str = 'little'):
        self.host = host
        self.port = port
        self.endian = endian
        self.thread = None
        self._stop = threading.Event()
        self._ready = threading.Event()

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self._stop.set()
        if self.thread:
            self.thread.join(timeout=2.0)

    def _run(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind((self.host, self.port))
                self.port = s.getsockname()[1]
                s.listen(1)
                self._ready.set()
                s.settimeout(5.0)
                try:
                    conn, _ = s.accept()
                except socket.timeout:
                    return
                with conn:
                    conn.settimeout(1.0)
                    # Header fields as expected by the node
                    msg_type = 15
                    comm_type = 1
                    reply_type = 0
                    payload = [1, 0, 0, 2, 3, 3276, 1638, 6553, 100]
                    # length counts bytes after this field, i.e. 12(header sans length)+36(payload)=48
                    length = 12 + 9 * 4
                    if self.endian == 'little':
                        hpack = '<IIII'
                        ipack = '<i'
                    else:
                        hpack = '>IIII'
                        ipack = '>i'
                    header = struct.pack(hpack, length, msg_type, comm_type, reply_type)
                    body = b''.join(struct.pack(ipack, v) for v in payload)
                    # Stream several frames
                    for _ in range(20):
                        if self._stop.is_set():
                            break
                        try:
                            conn.sendall(header + body)
                        except Exception:
                            break
                        time.sleep(0.05)
        except Exception:
            # Swallow exceptions to avoid noisy test failures from server teardown
            pass


@pytest.mark.launch_test
def generate_test_description():
    # Start the mock server first so the node can connect
    server = TcpStateServer(port=0, endian='little')
    server.start()
    assert server._ready.wait(timeout=2.0), 'Mock server failed to bind to a port'

    node = launch_ros.actions.Node(
        package='fanuc_driver',
        executable='weld_state_node_tcp',
        name='weld_state_node_tcp_test',
        output='screen',
        parameters=[{
            'robot_ip': '127.0.0.1',
            'robot_port': server.port,
            'byte_order': 'little',
            'debug': False,
            'dump_raw': False,
            'expected_msg_type': 15,
            'expected_comm_type': 1,
            'payload_fields': 9,
        }]
    )

    ld = launch.LaunchDescription([
        node,
        launch_testing.actions.ReadyToTest(),
    ])
    return ld, {'server': server}


def test_state_message_parsed(server):
    """Subscribe to /weld_state and assert parsed fields."""
    rclpy.init()
    try:
        node = rclpy.create_node('weld_state_node_tcp_test_client')
        result = {'msg': None}

        def cb(msg: WeldState):
            result['msg'] = msg

        sub = node.create_subscription(WeldState, '/weld_state', cb, 10)

        # Wait for publisher presence to reduce flakiness
        start = time.time()
        while sub.get_publisher_count() == 0 and time.time() - start < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Spin until we receive one message or timeout
        start = time.time()
        while result['msg'] is None and time.time() - start < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)

        assert result['msg'] is not None, 'Did not receive /weld_state message in time'
        msg = result['msg']

        # Validate booleans derived from payload[0..2]
        assert msg.arc_ok is True
        assert msg.power_err is False
        assert msg.depos_di is False

        # Validate raw int16 fields (cast is applied by the node)
        assert msg.ewm_err == 2
        assert msg.warning_state == 3
        assert msg.act_voltage == 3276
        assert msg.act_current == 1638
        assert msg.act_wire_spd == 6553
        assert msg.motor_current == 100
    finally:
        rclpy.shutdown()
        server.stop()


#!/usr/bin/env python3
"""
Integration test for `weld_state_node_simple`.

This test launches the node against a mock server that emits both
SimpleMessage payload-only frames (length=32) and standard frames with
sequence (length=53). It verifies `/weld_state` output matches payload.
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


class DualFormatServer:
    def __init__(self, host='127.0.0.1', port=0, endian='little'):
        self.host = host
        self.port = port
        self.endian = endian
        self._ready = threading.Event()
        self._stop = threading.Event()
        self.thread = None

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
                    msg_type, comm_type, reply_type = 15, 1, 0
                    payload = [1, 0, 0, 2, 3, 3276, 1638, 6553, 100]
                    if self.endian == 'little':
                        ipack = '<i'
                        h32 = '<IIII'
                    else:
                        ipack = '>i'
                        h32 = '>IIII'
                    # First: payload-only (length=32)
                    header = struct.pack(h32, 32, msg_type, comm_type, reply_type)
                    body = b''.join(struct.pack(ipack, v) for v in payload)
                    try:
                        conn.sendall(header + body)
                    except Exception:
                        return
                    time.sleep(0.05)
                    # Second: standard (length=53) = 4(seq)+36(payload); here we set length to 53 so node expects seq+payload
                    header2 = struct.pack(h32, 53, msg_type, comm_type, reply_type)
                    seq = struct.pack('<I' if self.endian == 'little' else '>I', 42)
                    try:
                        conn.sendall(header2 + seq + body)
                    except Exception:
                        return
                    # Keep connection a bit
                    time.sleep(0.1)
        except Exception:
            pass


@pytest.mark.launch_test
def generate_test_description():
    server = DualFormatServer()
    server.start()
    assert server._ready.wait(timeout=2.0)

    node = launch_ros.actions.Node(
        package='fanuc_driver',
        executable='weld_state_node_simple',
        name='weld_state_node_simple_test',
        output='screen',
        parameters=[{
            'robot_ip': '127.0.0.1',
            'robot_port': server.port,
            'byte_order': 'little',
            'debug': False,
            'payload_only_length': 32,
            'standard_format_length': 53,
            'expected_msg_type': 15,
            'expected_comm_type': 1,
            'num_weld_fields': 9,
        }]
    )

    ld = launch.LaunchDescription([
        node,
        launch_testing.actions.ReadyToTest(),
    ])
    return ld, {'server': server}


def test_simplemessage_parsing(server):
    rclpy.init()
    try:
        node = rclpy.create_node('weld_state_node_simple_test_client')
        got = []

        def cb(msg: WeldState):
            got.append(msg)

        sub = node.create_subscription(WeldState, '/weld_state', cb, 10)
        start = time.time()
        while sub.get_publisher_count() == 0 and time.time() - start < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)

        start = time.time()
        while len(got) < 2 and time.time() - start < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)

        assert len(got) >= 2, 'Expected at least two messages from dual-format server'
        # Validate fields for at least the first message
        m = got[0]
        assert m.arc_ok is True
        assert m.power_err is False
        assert m.depos_di is False
        assert m.ewm_err == 2
        assert m.warning_state == 3
        assert m.act_voltage == 3276
        assert m.act_current == 1638
        assert m.act_wire_spd == 6553
        assert m.motor_current == 100
    finally:
        rclpy.shutdown()
        server.stop()


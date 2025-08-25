#!/usr/bin/env python3
"""
Integration test for `weld_command_node`.

This test launches the ROS 2 node with loopback TCP settings and verifies
that publishing one `WeldCommand` on `/weld_command` results in a correctly
serialized TCP packet to the configured robot endpoint.

Key checks:
- The packet header fields (message type, command type, reply type, length)
  match the expected constants.
- The payload order and values match the node's `WeldCommandPacket` layout.

To avoid flakiness due to port reuse, the mock TCP server binds to an
ephemeral port and communicates it back to the launch description.
"""
import os
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
from fanuc_driver.msg import WeldCommand


class TcpMockServer:
    """Simple one-shot TCP server that captures exactly one incoming packet.

    The server binds to the specified `host` and `port`. If `port` is 0, an
    ephemeral port is chosen by the OS and stored back into `self.port`.
    The server records the first packet received into `self.last_packet` and
    replies with a minimal ACK compatible with the node's expectation.
    """

    def __init__(self, host: str = '127.0.0.1', port: int = 0):
        self.host = host
        self.port = port
        self.thread = None
        self._stop = threading.Event()
        self._ready = threading.Event()
        self.last_packet = None

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self._stop.set()
        if self.thread:
            self.thread.join(timeout=2.0)

    def _run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # Bind to requested port or let OS choose an ephemeral one when port==0
            s.bind((self.host, self.port))
            # If ephemeral, record the chosen port and mark ready
            self.port = s.getsockname()[1]
            s.listen(1)
            self._ready.set()
            s.settimeout(5.0)
            try:
                conn, _ = s.accept()
            except socket.timeout:
                return
            with conn:
                # Expect one packet: 15 uint32 (see WeldCommandPacket)
                expected_bytes = 15 * 4
                data = b''
                conn.settimeout(2.0)
                while len(data) < expected_bytes:
                    try:
                        chunk = conn.recv(expected_bytes - len(data))
                        if not chunk:
                            break
                        data += chunk
                    except socket.timeout:
                        break
                self.last_packet = data
                # Send minimal ACK: 5 uint32 (length, msg_type, comm_type, reply_type, seq)
                ack = struct.pack('<IIIII', expected_bytes, 14, 1, 1, 1)
                try:
                    conn.sendall(ack)
                except Exception:
                    pass


@pytest.mark.launch_test
def generate_test_description():
    """Launch description factory for launch_testing.

    Starts a local mock TCP server to capture the node's outgoing packet.
    The server binds to an ephemeral port, then we pass that port into the
    node parameters to ensure no port conflicts.
    """
    # Start mock server and wait until it has bound and exposed its port
    server = TcpMockServer(port=0)
    server.start()
    assert server._ready.wait(timeout=2.0), 'Mock server failed to bind to a port'

    # Launch weld_command_node with localhost params using the chosen port
    node = launch_ros.actions.Node(
        package='fanuc_driver',
        executable='weld_command_node',
        name='weld_command_node_test',
        output='screen',
        parameters=[{
            'robot_ip': '127.0.0.1',
            'robot_port': server.port,
            'byte_order': 'little',
            'enable_heartbeat': False,
        }]
    )

    ld = launch.LaunchDescription([
        node,
        # Signal launch_testing that everything is ready and tests may run
        launch_testing.actions.ReadyToTest(),
    ])
    return ld, {'server': server}


def test_packet_transmission(server):
    """Publish one WeldCommand and assert the outgoing TCP packet fields.

    The `weld_command_node` transforms the message into a packed C-struct with
    fixed layout and little-endian uint32 fields. This test reconstructs those
    fields and validates both header and payload values.
    """
    rclpy.init()
    try:
        node = rclpy.create_node('weld_command_node_test_client')
        pub = node.create_publisher(WeldCommand, '/weld_command', 10)

        # Wait for the node under test to subscribe so we don't drop the publish
        start = time.time()
        while pub.get_subscription_count() == 0 and time.time() - start < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Publish one command with explicit values for every field
        msg = WeldCommand()
        msg.target_wire_spd = 100
        msg.correction_val = 1
        msg.dyn_setting = 2
        msg.operation_mode = 3
        msg.std_pulse_val = 4
        msg.program_number = 5
        msg.arc_start_cmd = 6
        msg.gas_control = 7
        msg.jog_feed_cmd = 8
        msg.jog_retract_cmd = 9
        pub.publish(msg)

        # Allow time for the TCP connect+send path to execute
        time.sleep(0.8)

        # Validate the captured packet
        assert server.last_packet is not None, 'No packet captured by mock server'
        data = server.last_packet
        assert len(data) >= 15 * 4

        # Unpack little-endian 15 uint32 fields
        fields = struct.unpack('<' + 'I' * 15, data[:15 * 4])
        length, msg_type, comm_type, reply_type, seq_nr = fields[:5]
        payload = fields[5:]

        # Header constants from the node implementation
        assert msg_type == 14          # RI_MT_WELDCMD
        assert comm_type == 1          # RI_CT_SVCREQ
        assert reply_type == 0         # RI_RT_INVAL for requests
        assert length == 15 * 4        # total struct size in bytes

        # Payload order must match WeldCommandPacket in the node
        assert payload == (
            100, 1, 2, 3, 4, 5, 6, 7, 8, 9
        )
    finally:
        rclpy.shutdown()
        server.stop()


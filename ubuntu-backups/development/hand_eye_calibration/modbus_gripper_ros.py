"""ROS 2 driver bridge for the custom tool-RS485 Modbus-RTU gripper.

Values are taken from ``windows_gripper_code/control/arm/arm_control.py``.
This is intentionally separate from RealMan's standard gripper topics.
"""

import time

import rclpy
from rm_ros_interfaces.msg import Modbusrtuwriteparams, RS485params
from std_msgs.msg import Bool, UInt16


def u32_bytes(value):
    """Encode an unsigned 32-bit register value as four Modbus bytes."""
    value = int(value)
    if not 0 <= value <= 0xFFFFFFFF:
        raise ValueError(f"Modbus value outside uint32 range: {value}")
    return [
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF,
    ]


class ToolRs485ModbusGripper:
    """Use the RM driver's end-interface-board RS485 port (port 1)."""

    def __init__(self, node):
        self.node = node
        self.results = {}
        self.rs485_pub = node.create_publisher(
            RS485params, "/rm_driver/set_controller_rs485_mode_cmd", 10
        )
        self.voltage_pub = node.create_publisher(
            UInt16, "/rm_driver/set_tool_voltage_cmd", 10
        )
        self.write_pub = node.create_publisher(
            Modbusrtuwriteparams,
            "/rm_driver/write_modbus_rtu_registers_cmd",
            10,
        )
        self._subscriptions = [
            node.create_subscription(
                Bool,
                "/rm_driver/set_controller_rs485_mode_result",
                self._result_callback("rs485"),
                10,
            ),
            node.create_subscription(
                Bool,
                "/rm_driver/set_tool_voltage_result",
                self._result_callback("voltage"),
                10,
            ),
            node.create_subscription(
                Bool,
                "/rm_driver/write_modbus_rtu_registers_result",
                self._result_callback("write"),
                10,
            ),
        ]

    def _result_callback(self, name):
        def callback(message):
            self.results[name] = bool(message.data)

        return callback

    def _wait_for_subscription(self, publisher, label, timeout_s):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if publisher.get_subscription_count() >= 1:
                return
        raise RuntimeError(f"No rm_driver subscription for {label}")

    def _wait_result(self, key, label, timeout_s):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if key in self.results:
                if self.results[key]:
                    print(f"Gripper PASS: {label}")
                    return
                raise RuntimeError(f"Gripper rejected: {label}; inspect rm_driver log")
        raise RuntimeError(f"Gripper timeout: {label}; inspect rm_driver log")

    def _write_registers(self, cfg, address, num, data, label):
        timeout_s = float(cfg["result_timeout_s"])
        self._wait_for_subscription(self.write_pub, label, timeout_s)
        self.results.pop("write", None)
        message = Modbusrtuwriteparams()
        message.address = int(address)
        message.device = int(cfg["device_address"])
        message.type = int(cfg["port"])
        message.num = int(num)
        message.data = [int(value) for value in data]
        self.write_pub.publish(message)
        self._wait_result("write", label, timeout_s)

    def configure(self, cfg):
        """Configure tool RS485 and 24 V without causing mechanical motion."""
        timeout_s = float(cfg["result_timeout_s"])
        self._wait_for_subscription(self.rs485_pub, "RS485 setup", timeout_s)
        self.results.pop("rs485", None)
        message = RS485params()
        message.mode = int(cfg["port"])
        message.baudrate = int(cfg["baudrate"])
        self.rs485_pub.publish(message)
        self._wait_result("rs485", "tool RS485 Modbus setup", timeout_s)

        self._wait_for_subscription(self.voltage_pub, "tool voltage", timeout_s)
        self.results.pop("voltage", None)
        self.voltage_pub.publish(UInt16(data=int(cfg["tool_voltage"])))
        self._wait_result("voltage", "tool 24 V enable", timeout_s)

    def initialize(self, cfg):
        """Configure communication, then perform the mechanical zeroing sequence."""
        self.configure(cfg)
        self._write_registers(
            cfg,
            cfg["zero_speed_register"],
            2,
            u32_bytes(cfg["zero_speed"]),
            "set gripper zeroing speed",
        )
        # The verified Windows program waits here. The gripper controller
        # needs this settling time before accepting the zeroing command.
        speed_settle_s = 1.0
        print(f"Waiting {speed_settle_s:.1f} s after setting zeroing speed")
        deadline = time.monotonic() + speed_settle_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self._write_registers(
            cfg,
            cfg["zero_register"],
            1,
            [int(cfg["zero_value"])],
            "start gripper zeroing",
        )
        settle_s = float(cfg["zero_settle_s"])
        print(f"Waiting {settle_s:.1f} s for custom gripper zeroing")
        deadline = time.monotonic() + settle_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def set_opening_mm(self, cfg, opening_mm):
        opening_mm = float(opening_mm)
        if not 0.0 <= opening_mm <= 120.0:
            raise ValueError(f"Gripper opening must be within 0..120 mm, got {opening_mm}")
        steps = round(opening_mm * float(cfg["steps_per_mm"]))
        self._write_registers(
            cfg,
            cfg["position_register"],
            2,
            u32_bytes(steps),
            f"set gripper opening to {opening_mm:.1f} mm ({steps} steps)",
        )

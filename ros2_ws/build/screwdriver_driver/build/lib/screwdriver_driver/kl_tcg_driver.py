import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import serial


TORQUE_UNITS = {
    "0": "kgf.cm",
    "1": "N.m",
    "2": "lbf.in",
    "3": "kgf.m",
}


class KLTcgDriver(Node):
    def __init__(self):
        super().__init__("kl_tcg_driver")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        self.raw_pub = self.create_publisher(String, "/screwdriver/raw_data", 10)
        self.device_status_pub = self.create_publisher(String, "/screwdriver/device_status", 10)
        self.live_torque_pub = self.create_publisher(Float32, "/screwdriver/live_torque", 10)
        self.final_result_pub = self.create_publisher(String, "/screwdriver/final_result", 10)
        self.human_status_pub = self.create_publisher(String, "/screwdriver/human_status", 10)

        port = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=0.1,
            )
            self.get_logger().info(f"Connected to KL-TCG on {port} at {baudrate}")
        except Exception as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            self.ser = None

        self.timer = self.create_timer(0.02, self.read_serial)

    def read_serial(self):
        if self.ser is None:
            return

        try:
            line = self.ser.readline().decode(errors="ignore").strip()
            if not line:
                return

            self.publish_string(self.raw_pub, line)

            fields = self.parse_frame(line)
            if not fields:
                return

            command = fields[0]

            if command == "REQ100":
                self.handle_req100(fields, line)
            elif command == "DATA101":
                self.handle_data101(fields, line)
            elif command == "DATA100":
                self.handle_data100(fields, line)
            else:
                self.publish_string(
                    self.human_status_pub,
                    f"Unknown KL-TCG frame ({command}): {line}"
                )

        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

    def parse_frame(self, line):
        clean = line.strip().strip("{}")
        if not clean:
            return []
        return clean.split(",")

    def handle_req100(self, fields, raw_line):
        try:
            device_id = fields[11]
            tool_sn = fields[12]
            device_sn = fields[13]
            operation_mode = fields[14]
            job = fields[16]
            sequence = fields[17]
            program = fields[19]
            tool_connected = fields[21]
            tool_enabled = fields[24]
            tool_stop_status = fields[25]
            screw_count = fields[26]

            tool_connected_text = "tool connected" if tool_connected == "1" else "tool not connected"
            tool_enabled_text = "tool enabled" if tool_enabled == "1" else "tool disabled"

            decoded = (
                f"KL-TCG status (REQ100): controller alive | "
                f"{tool_connected_text} | "
                f"{tool_enabled_text} | "
                f"job={job} | "
                f"sequence={sequence} | "
                f"program={program} | "
                f"screws={screw_count} | "
                f"stop_status={tool_stop_status}"
            )

            self.publish_string(self.device_status_pub, decoded)
            self.publish_string(self.human_status_pub, decoded)

        except IndexError:
            self.publish_string(
                self.human_status_pub,
                f"Incomplete REQ100 frame: {raw_line}"
            )

    def handle_data101(self, fields, raw_line):
        try:
            fastening_time = float(fields[1])
            torque = float(fields[2])

            torque_msg = Float32()
            torque_msg.data = torque
            self.live_torque_pub.publish(torque_msg)

            decoded = (
                f"Live torque received (DATA101): "
                f"time={fastening_time:.3f}s, "
                f"torque={torque}"
            )

            self.publish_string(self.human_status_pub, decoded)

        except (IndexError, ValueError):
            self.publish_string(
                self.human_status_pub,
                f"Could not decode DATA101 frame: {raw_line}"
            )

    def handle_data100(self, fields, raw_line):
        try:
            device_count = fields[13]
            job = fields[14]
            sequence = fields[15]
            program = fields[16]
            program_name = fields[17]
            selected_tool = fields[18]
            torque = fields[19]
            torque_unit_code = fields[20]
            torque_unit = TORQUE_UNITS.get(torque_unit_code, "unknown")
            fastening_time = fields[21]
            fastening_thread = fields[22]
            screw_count = fields[23]
            inc_dec = fields[24]
            status = fields[25]
            tool_stop_status = fields[26]

            decoded = (
                f"Final result received (DATA100): "
                f"device_count={device_count}, "
                f"job={job}, "
                f"sequence={sequence}, "
                f"program={program}, "
                f"program_name={program_name}, "
                f"selected_tool={selected_tool}, "
                f"torque={torque} {torque_unit}, "
                f"fastening_time={fastening_time}s, "
                f"thread_count={fastening_thread}, "
                f"screw_count={screw_count}, "
                f"inc_dec={inc_dec}, "
                f"status={status}, "
                f"tool_stop_status={tool_stop_status}"
            )

            self.publish_string(self.final_result_pub, decoded)
            self.publish_string(self.human_status_pub, decoded)

        except IndexError:
            self.publish_string(
                self.human_status_pub,
                f"Incomplete DATA100 frame: {raw_line}"
            )

    def publish_string(self, publisher, text):
        msg = String()
        msg.data = text
        publisher.publish(msg)
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = KLTcgDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
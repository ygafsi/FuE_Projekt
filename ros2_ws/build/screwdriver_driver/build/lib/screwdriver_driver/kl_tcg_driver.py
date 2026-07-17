import datetime
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import serial


TORQUE_UNITS = {
    "0": "kgf.cm",
    "1": "N.m",
    "2": "lbf.in",
    "3": "kgf.m",
}

TOOL_STOP_STATUS = {
    "0": "none",
    "1": "NS - fastening error",
    "2": "AS",
    "3": "E3 - undervoltage",
    "4": "E4 - tool overtemperature",
    "5": "E5 - motor stall",
    "7": "E7 - torque sensor abnormal",
    "8": "E8 - temperature abnormal",
    "9": "E9 - motor running too long",
    "A": "EPC",
    "B": "ESC",
    "C": "ES - screwdriver communication error",
    "D": "Er - gate input abnormal",
    "E": "C1 - waiting for GATE",
    "F": "C2 - waiting for two GATE signals",
    "G": "C4 - gate + OKALL lock active",
    "H": "C5 - double gate + OKALL lock active",
    "I": "EOC - calibration required",
    "J": "BS - waiting for barcode",
}


class KLTcgDriver(Node):
    def __init__(self):
        super().__init__("kl_tcg_driver")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("device_name", 0)
        self.declare_parameter("target_job", 1)
        self.declare_parameter("target_sequence", 1)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.device_name = int(self.get_parameter("device_name").value)

        self.state_pub = self.create_publisher(String, "/screwdriver/state", 10)
        self.result_pub = self.create_publisher(String, "/screwdriver/result", 10)
        self.live_torque_pub = self.create_publisher(String, "/screwdriver/live_torque", 10)
        self.raw_frame_pub = self.create_publisher(String, "/screwdriver/raw_frame", 10)
        self.events_pub = self.create_publisher(String, "/screwdriver/events", 10)

        self._instruction_number = 1
        self._serial_lock = threading.Lock()

        self.srv_read_config = self.create_service(
            Trigger,
            "/screwdriver/read_controller_config",
            self.cb_read_controller_config,
        )

        self.srv_read_tool = self.create_service(
            Trigger,
            "/screwdriver/read_tool_status",
            self.cb_read_tool_status,
        )

        self.srv_execute_job = self.create_service(
            Trigger,
            "/screwdriver/execute_job",
            self.cb_execute_job,
        )

        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=0.1,
            )
            self.publish_event(f"Connected to KL-TCG on {self.port} at {self.baudrate} baud")
        except Exception as e:
            self.ser = None
            self.publish_event(f"ERROR: Could not open serial port: {e}")

        self.timer = self.create_timer(0.02, self.read_serial)

    def read_serial(self):
        if self.ser is None:
            return

        try:
            with self._serial_lock:
                line = self.ser.readline().decode(errors="ignore").strip()

            if not line:
                return

            self.publish_string(self.raw_frame_pub, line, log=False)

            fields = self.parse_frame(line)
            if not fields:
                return

            command = fields[0]

            if command == "REQ100":
                self.handle_req100(fields, line)
            elif command == "DATA100":
                self.handle_data100(fields, line)
            elif command == "DATA101":
                self.handle_data101(fields, line)
            elif command.startswith("ANS"):
                self.publish_event(f"Controller answer: {command} | raw={line}")
            else:
                self.publish_event(f"Unknown frame: command={command}, raw={line}")

        except Exception as e:
            self.publish_event(f"ERROR: Serial read error: {e}")

    def parse_frame(self, line):
        clean = line.strip().strip("{}")
        if not clean:
            return []
        return clean.split(",")

    def handle_req100(self, fields, raw_line):
        try:
            operation_mode = fields[14]
            job = fields[16]
            sequence = fields[17]
            program = fields[19]
            tool_connected = fields[21]
            tool_enabled = fields[24]
            stop_code = fields[25]
            screw_count = fields[26]

            stop_text = TOOL_STOP_STATUS.get(stop_code, "unknown")

            state = (
                "STATE\n"
                "-----\n"
                "Controller : connected\n"
                f"Tool       : {self.decode_connected(tool_connected)}\n"
                f"Enabled    : {self.decode_yes_no(tool_enabled)}\n"
                f"Mode       : {self.decode_operation_mode(operation_mode)}\n"
                f"Job/Seq/TP : {job}/{sequence}/{program}\n"
                f"Screws     : {screw_count}\n"
                f"Stop       : {stop_code} ({stop_text})"
            )

            self.publish_string(self.state_pub, state, log=False)

        except IndexError:
            self.publish_event(f"Incomplete REQ100 frame: {raw_line}")

    def handle_data100(self, fields, raw_line):
        try:
            cycle_count = int(fields[13])
            job = fields[14]
            sequence = fields[15]
            program = fields[16]
            torque = self.to_float(fields[19])
            torque_unit = TORQUE_UNITS.get(fields[20], "unknown")
            fastening_time = self.to_float(fields[21])
            rotations = self.to_float(fields[22])
            screw_count = fields[23]
            status = self.clean_text(fields[25])
            stop_code = fields[26].strip("}")
            stop_text = TOOL_STOP_STATUS.get(stop_code, "unknown")

            state = (
                "STATE\n"
                "-----\n"
                "Controller : connected\n"
                "Tool       : connected\n"
                f"Job/Seq/TP : {job}/{sequence}/{program}\n"
                f"Screws     : {screw_count}\n"
                f"Stop       : {stop_code} ({stop_text})"
            )

            result = (
                "LAST RESULT\n"
                "-----------\n"
                f"Result     : {status}\n"
                f"Torque     : {torque:.3f} {torque_unit}\n"
                f"Time       : {fastening_time:.3f} s\n"
                f"Rotations  : {rotations:.1f}\n"
                f"Cycle Nr.  : {cycle_count}"
            )

            self.publish_string(self.state_pub, state, log=False)
            self.publish_string(self.result_pub, result)

        except (IndexError, ValueError):
            self.publish_event(f"Could not decode DATA100 frame: {raw_line}")

    def handle_data101(self, fields, raw_line):
        try:
            fastening_time = self.to_float(fields[1])
            torque = self.to_float(fields[2])
            
            live = (
                "LIVE TORQUE\n"
                "-----------\n"
                f"Torque : {torque:.4f} Nm\n"
                f"Time   : {fastening_time:.3f} s"
            )

            self.publish_string(self.live_torque_pub, live, log=False)

        except (IndexError, ValueError):
            self.publish_event(f"Could not decode DATA101 frame: {raw_line}")

    def cb_read_controller_config(self, request, response):
        frame = self.build_command_frame(108)
        ok, info = self.send_frame(frame)

        response.success = ok
        response.message = f"CMD108 sent: {info}" if ok else info
        return response

    def cb_read_tool_status(self, request, response):
        frame = self.build_command_frame(116)
        ok, info = self.send_frame(frame)

        response.success = ok
        response.message = f"CMD116 sent: {info}" if ok else info
        return response

    def cb_execute_job(self, request, response):
        job = int(self.get_parameter("target_job").value)
        sequence = int(self.get_parameter("target_sequence").value)

        if not (1 <= job <= 50):
            response.success = False
            response.message = f"Invalid job={job}. Must be 1..50."
            return response

        if not (1 <= sequence <= 50):
            response.success = False
            response.message = f"Invalid sequence={sequence}. Must be 1..50."
            return response

        frame = self.build_command_frame(
            104,
            extra_fields=[f"{job:02d}", f"{sequence:02d}"],
        )

        ok, info = self.send_frame(frame)

        response.success = ok
        response.message = (
            f"CMD104 sent for Job={job}, Sequence={sequence}: {info}"
            if ok
            else info
        )
        return response

    def build_command_frame(self, command_code, extra_fields=None):
        if extra_fields is None:
            extra_fields = []

        t = self.time_fields()
        instruction = self.next_instruction_number()

        fields = [
            f"CMD{command_code}",
            t["year"],
            t["month"],
            t["day"],
            t["hour"],
            t["minute"],
            t["second"],
            t["checksum"],
            t["keycode"],
            str(self.device_name),
            *extra_fields,
            str(instruction),
        ]

        return "{" + ",".join(fields) + ",}"

    def send_frame(self, frame):
        if self.ser is None:
            return False, "serial port not connected"

        try:
            data = (frame + "\r\n").encode("ascii")

            with self._serial_lock:
                self.ser.write(data)

            self.publish_event(f"Sent command: {frame}")
            return True, frame

        except Exception as e:
            return False, f"serial write error: {e}"

    def next_instruction_number(self):
        number = self._instruction_number
        self._instruction_number += 1

        if self._instruction_number > 255:
            self._instruction_number = 1

        return number

    def time_fields(self):
        now = datetime.datetime.now()

        checksum = (
            now.year
            + now.month
            + now.day
            + now.hour
            + now.minute
            + now.second
        )
        keycode = checksum + 5438

        return {
            "year": f"{now.year:04d}",
            "month": f"{now.month:02d}",
            "day": f"{now.day:02d}",
            "hour": f"{now.hour:02d}",
            "minute": f"{now.minute:02d}",
            "second": f"{now.second:02d}",
            "checksum": f"{checksum % 10000:04d}",
            "keycode": f"{keycode % 10000:04d}",
        }

    def publish_string(self, publisher, text, log=True):
        msg = String()
        msg.data = text
        publisher.publish(msg)
        if log:
            self.get_logger().info(text)

    def publish_event(self, text):
        self.publish_string(self.events_pub, text)

    def clean_text(self, text):
        return text.strip().strip("}").replace("_", "")

    def to_float(self, text):
        return float(text.strip().strip("}"))

    def decode_operation_mode(self, code):
        return {
            "0": "ADV",
            "1": "STD",
            "2": "ALI",
            "3": "SET",
        }.get(code, f"unknown ({code})")

    def decode_connected(self, code):
        return "connected" if code == "1" else "not connected"

    def decode_yes_no(self, code):
        return "yes" if code == "1" else "no"


def main(args=None):
    rclpy.init(args=args)
    node = KLTcgDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
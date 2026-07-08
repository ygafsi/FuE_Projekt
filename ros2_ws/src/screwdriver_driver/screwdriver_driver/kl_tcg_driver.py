import datetime
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger
import serial


# ============================================================
# Command setpoint (default). Edit these to test other values,
# then rebuild and relaunch. Applied once to step 1 at startup.
# ============================================================
DEFAULT_DIRECTION = "ccw"     # "ccw" (unscrew) or "cw" (screw)
DEFAULT_SPEED_RPM = 300       # target speed
DEFAULT_TORQUE_NM = 0.30      # target torque (N.m); inert on step 1 (thread mode)
DEFAULT_TURNS = 5.0           # number of turns before stopping (thread mode)
APPLY_ON_START = True       # write the command to step 1 once at startup


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

# ANS108 uses a different torque-unit encoding than DATA100. Do not mix them.
ANS108_TORQUE_UNITS = {"0": "Kgf.m", "1": "N.m", "2": "Kgf.cm", "3": "Lbf.in"}
GATE_MODE = {"0": "OFF", "1": "ONCE", "2": "TWICE"}
START_SIGNAL_MODE = {"0": "MOTOR", "1": "TRIGGER"}
SCREWDRIVER_ENABLE = {"0": "Disable Both", "1": "Enable FWD", "2": "Enable REV", "3": "Enable Both"}
ANS108_MODE = {"0": "ADV", "1": "STD", "2": "ALI", "3": "SET"}

# CMD121 target torque is in Kgf.cm, Jussi thinks N.m. Factor to verify on HW.
NM_TO_KGFCM = 10.197


class KLTcgDriver(Node):
    def __init__(self):
        super().__init__("kl_tcg_driver")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("device_name", 0)

        # Job/sequence to select, and the step it runs. On this controller:
        # job 1 -> program 1 -> step 1 (thread mode, 5 turns). Writing targets
        # step 1 so the command actually changes what the tool does.
        self.declare_parameter("target_job", 1)
        self.declare_parameter("target_sequence", 1)
        self.declare_parameter("step_number", 1)

        # Command values, defaulting to the constants above.
        self.declare_parameter("write_speed", DEFAULT_SPEED_RPM)
        self.declare_parameter("write_direction", 0 if DEFAULT_DIRECTION == "ccw" else 1)
        self.declare_parameter("write_torque_nm", DEFAULT_TORQUE_NM)
        self.declare_parameter("write_turns", DEFAULT_TURNS)

        # Manual read via /screwdriver/read_setting.
        self.declare_parameter("read_select", 1)      # 1=step, 2=program, 3=job
        self.declare_parameter("read_number", 1)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.device_name = int(self.get_parameter("device_name").value)

        # Real telemetry out (what Jussi and dashboards consume).
        self.live_torque_pub = self.create_publisher(Float32, "/screwdriver/live_torque", 10)
        self.live_rpm_pub = self.create_publisher(Float32, "/screwdriver/live_rpm", 10)
        self.final_torque_pub = self.create_publisher(Float32, "/screwdriver/final_torque", 10)
        self.final_result_pub = self.create_publisher(String, "/screwdriver/final_result", 10)
        self.device_status_pub = self.create_publisher(String, "/screwdriver/device_status", 10)
        self.human_status_pub = self.create_publisher(String, "/screwdriver/human_status", 10)
        self.config_pub = self.create_publisher(String, "/screwdriver/controller_config", 10)
        self.raw_frame_pub = self.create_publisher(String, "/screwdriver/raw_frame", 10)
        self.events_pub = self.create_publisher(String, "/screwdriver/events", 10)
        self.command_pub = self.create_publisher(String, "/screwdriver/command", 10)

        self._instruction_number = 1
        self._serial_lock = threading.Lock()
        self._rx_buffer = ""

        self.srv_read_config = self.create_service(
            Trigger, "/screwdriver/read_controller_config", self.cb_read_controller_config
        )
        self.srv_read_tool = self.create_service(
            Trigger, "/screwdriver/read_tool_status", self.cb_read_tool_status
        )
        self.srv_execute_job = self.create_service(
            Trigger, "/screwdriver/execute_job", self.cb_execute_job
        )
        self.srv_read_setting = self.create_service(
            Trigger, "/screwdriver/read_setting", self.cb_read_setting
        )
        self.srv_write_step = self.create_service(
            Trigger, "/screwdriver/write_step", self.cb_write_step
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

        # Apply the command setpoint once, shortly after startup.
        if APPLY_ON_START:
            self._apply_timer = self.create_timer(1.5, self.apply_command_once)

    # ---------------- Apply the command once at startup ----------------
    def apply_command_once(self):
        self._apply_timer.cancel()
        step = int(self.get_parameter("step_number").value)
        speed = int(self.get_parameter("write_speed").value)
        direction = int(self.get_parameter("write_direction").value)
        torque_nm = float(self.get_parameter("write_torque_nm").value)
        turns = float(self.get_parameter("write_turns").value)
        torque_kgfcm = self.nm_to_kgfcm(torque_nm)
        dir_word = "ccw" if direction == 0 else "cw"

        cmd = f"dir={dir_word} speed={speed} turns={turns} torque={torque_nm}Nm"
        self.publish_string(self.command_pub, cmd)

        ok, info = self.write_step(step, speed, direction, torque_kgfcm, turns)
        if not ok:
            self.publish_event(f"startup apply failed: {info}")
            return
        self.select_job()
        self.publish_human(f"Applied command on step {step}: {cmd}")

    # ---------------- Serial read: {...} delimiter buffering ----------------
    def read_serial(self):
        if self.ser is None:
            return
        try:
            n = self.ser.in_waiting or 0
            if n:
                self._rx_buffer += self.ser.read(n).decode(errors="ignore")
        except Exception as e:
            self.publish_event(f"ERROR: Serial read error: {e}")
            return

        while True:
            start = self._rx_buffer.find("{")
            if start == -1:
                self._rx_buffer = ""
                break
            end = self._rx_buffer.find("}", start)
            if end == -1:
                self._rx_buffer = self._rx_buffer[start:]
                break
            frame = self._rx_buffer[start:end + 1]
            self._rx_buffer = self._rx_buffer[end + 1:]
            self.process_frame(frame)

    def process_frame(self, frame):
        self.publish_string(self.raw_frame_pub, frame, log=False)
        fields = self.parse_frame(frame)
        if not fields:
            return
        command = fields[0]

        if command == "REQ100":
            self.handle_req100(fields, frame)
        elif command == "DATA100":
            self.handle_data100(fields, frame)
        elif command == "DATA101":
            self.handle_data101(fields, frame)
        elif command in ("ANS108", "108"):
            self.handle_ans108(fields, frame)
        elif command in ("ANS124", "124"):
            self.handle_ans124(fields, frame)
        elif command.startswith("ANS"):
            self.publish_event(f"Controller answer: {command} | raw={frame}")
        else:
            self.publish_event(f"Unknown frame: command={command}, raw={frame}")

    def parse_frame(self, line):
        clean = line.strip().strip("{}")
        if not clean:
            return []
        return clean.split(",")

    # ---------------- Decoders ----------------
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

            status = (
                "device=connected "
                f"tool={self.decode_connected(tool_connected)} "
                f"enabled={self.decode_yes_no(tool_enabled)} "
                f"mode={self.decode_operation_mode(operation_mode)} "
                f"job/seq/tp={job}/{sequence}/{program} "
                f"screws={screw_count} "
                f"stop={stop_code}({stop_text})"
            )
            self.publish_string(self.device_status_pub, status, log=False)
        except IndexError:
            self.publish_event(f"Incomplete REQ100 frame: {raw_line}")

    def handle_data100(self, fields, raw_line):
        try:
            torque = self.to_float(fields[19])
            fastening_time = self.to_float(fields[21])
            rotations = self.to_float(fields[22])
            status = self.clean_text(fields[25])

            self.publish_float(self.final_torque_pub, torque)
            self.publish_string(self.final_result_pub, status)
            self.publish_human(
                f"Result={status}, torque={torque:.3f} Nm, "
                f"time={fastening_time:.3f} s, turns={rotations:.1f}"
            )
        except (IndexError, ValueError):
            self.publish_event(f"Could not decode DATA100 frame: {raw_line}")

        self.send_cmd100_ack()

    def handle_data101(self, fields, raw_line):
        try:
            torque = self.to_float(fields[2])
            self.publish_float(self.live_torque_pub, torque)
        except (IndexError, ValueError):
            self.publish_event(f"Could not decode DATA101 torque: {raw_line}")
            return
        try:
            rpm = self.to_float(fields[3])
            self.publish_float(self.live_rpm_pub, rpm)
        except (IndexError, ValueError):
            pass

    def handle_ans108(self, fields, raw_line):
        try:
            unit_raw = fields[12]
            gate_raw = fields[13]
            mode_raw = fields[14]
            start_raw = fields[15]
            enable_raw = fields[17]

            config = (
                "CONTROLLER CONFIG (ANS108)\n"
                "--------------------------\n"
                f"Gate mode       : {gate_raw} ({GATE_MODE.get(gate_raw, 'unknown')})\n"
                f"Start signal    : {start_raw} ({START_SIGNAL_MODE.get(start_raw, 'unknown')})\n"
                f"Mode            : {mode_raw} ({ANS108_MODE.get(mode_raw, 'unknown')})\n"
                f"Direction autor.: {enable_raw} ({SCREWDRIVER_ENABLE.get(enable_raw, 'unknown')})\n"
                f"Torque unit     : {unit_raw} ({ANS108_TORQUE_UNITS.get(unit_raw, 'unknown')})"
            )
            self.publish_string(self.config_pub, config)
        except IndexError:
            self.publish_event(f"ANS108 too short: {raw_line}")

    def handle_ans124(self, fields, raw_line):
        try:
            kind = fields[9]   # 1=step, 2=program, 3=job
        except IndexError:
            self.publish_event(f"ANS124 too short: {raw_line}")
            return

        if kind == "1":
            block = (
                "READBACK STEP (ANS124)\n----------------------\n"
                f"Step number  : {fields[10]}\n"
                f"Speed (RPM)  : {fields[12]}\n"
                f"Target cond  : {fields[13]} (1=thread, 2=torque)\n"
                f"Target torque: {fields[16]} (Kgf.cm)\n"
                f"Direction    : {fields[17]} (0=CCW/reverse, 1=CW/forward)"
            )
        elif kind == "2":
            block = (
                "READBACK PROGRAM (ANS124)\n-------------------------\n"
                f"Program number: {fields[10]}\n"
                f"Step 1 : {fields[13]}\n"
                f"Step 2 : {fields[14]}\n"
                f"Step 3 : {fields[15]}\n"
                f"Step 4 : {fields[16]}\n"
                f"Step 5 : {fields[17]}"
            )
        elif kind == "3":
            block = (
                "READBACK JOB (ANS124)\n---------------------\n"
                f"Job number  : {fields[10]}\n"
                f"Total JS    : {fields[11]}\n"
                f"JS          : {fields[12]}\n"
                f"Program (TP): {fields[13]}\n"
                f"TR          : {fields[14]}"
            )
        else:
            block = f"ANS124 unknown kind={kind}"

        self.publish_string(self.config_pub, block)

        dump = "ANS124 RAW FIELDS\n-----------------\n"
        for i, v in enumerate(fields):
            dump += f"[{i}] str{i + 1} = {v}\n"
        self.publish_string(self.config_pub, dump)

    # ---------------- Command services ----------------
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
        ok, info = self.select_job()
        response.success = ok
        response.message = info
        return response

    def cb_read_setting(self, request, response):
        select = int(self.get_parameter("read_select").value)
        number = int(self.get_parameter("read_number").value)
        if select not in (1, 2, 3):
            response.success = False
            response.message = f"read_select must be 1/2/3, got {select}"
            return response
        frame = self.build_command_frame(124, extra_fields=[str(select), str(number)])
        ok, info = self.send_frame(frame)
        response.success = ok
        response.message = f"CMD124 sent (select={select}, number={number}): {info}" if ok else info
        return response

    def cb_write_step(self, request, response):
        step = int(self.get_parameter("step_number").value)
        speed = int(self.get_parameter("write_speed").value)
        direction = int(self.get_parameter("write_direction").value)
        torque_nm = float(self.get_parameter("write_torque_nm").value)
        turns = float(self.get_parameter("write_turns").value)
        if direction not in (0, 1):
            response.success = False
            response.message = f"write_direction must be 0(CCW) or 1(CW), got {direction}"
            return response
        torque_kgfcm = self.nm_to_kgfcm(torque_nm)
        ok, info = self.write_step(step, speed, direction, torque_kgfcm, turns)
        response.success = ok
        response.message = (
            f"CMD121 step={step} speed={speed} dir={direction} "
            f"torque={torque_kgfcm}Kgf.cm: {info}" if ok else info
        )
        return response

    def select_job(self):
        job = int(self.get_parameter("target_job").value)
        sequence = int(self.get_parameter("target_sequence").value)
        if not (1 <= job <= 50):
            return False, f"Invalid job={job}. Must be 1..50."
        if not (1 <= sequence <= 50):
            return False, f"Invalid sequence={sequence}. Must be 1..50."
        frame = self.build_command_frame(104, extra_fields=[f"{job:02d}", f"{sequence:02d}"])
        ok, info = self.send_frame(frame)
        return ok, (f"CMD104 sent Job={job} Seq={sequence}: {info}" if ok else info)

    def write_step(self, step, speed, direction, torque_kgfcm, turns):
        if not (1 <= step <= 250):
            return False, f"step must be 1..250, got {step}"
        torque_kgfcm = max(0.0, min(99.0, torque_kgfcm))

        # Template = step 1 as read back (CMD124) on 2026-07-08. Step 1 is what
        # program 1 (job 1) actually runs. It is in THREAD mode (condition=1):
        # it turns 5 turns then stops, which is why cycles return OK. That mode
        # is preserved. Only speed (str13) and direction (str18) are driven by
        # the command. Torque (str17/str22) is written but, in thread mode, it
        # is NOT the stop condition, so it does not stop the rotation.
        extra = [
            str(step),               # str11 TS number
            "000001",                # str12 step name (step 1)
            str(speed),              # str13 speed              <- command
            "1",                     # str14 target condition (1=thread)
            "0.000",                 # str15 unused
            f"{turns:.1f}",          # str16 target thread (turns)   <- command
            f"{torque_kgfcm:.2f}",   # str17 target torque (Kgf.cm), inert here
            str(direction),          # str18 direction (0=CCW)  <- command
            "0.8",                   # str19 delay time
            "999.9",                 # str20 upper thread limit
            "0.0",                   # str21 lower thread limit
            f"{torque_kgfcm:.2f}",   # str22 upper torque limit
            "0.00",                  # str23 lower torque limit
            "+0",                    # str24 unused
            "10",                    # str25 unused
            "10",                    # str26 unused
            "0",                     # str27 unused
            "+0",                    # str28 unused
        ]
        frame = self.build_command_frame(121, extra_fields=extra)
        return self.send_frame(frame)

    def send_cmd100_ack(self):
        frame = self.build_command_frame(100)
        ok, info = self.send_frame(frame)
        if not ok:
            self.publish_event(f"CMD100 ack failed: {info}")

    # ---------------- Frame building ----------------
    def build_command_frame(self, command_code, extra_fields=None):
        if extra_fields is None:
            extra_fields = []
        t = self.time_fields()
        instruction = self.next_instruction_number()
        fields = [
            f"CMD{command_code}",
            t["year"], t["month"], t["day"], t["hour"], t["minute"], t["second"],
            t["checksum"], t["keycode"], str(self.device_name),
            *extra_fields, str(instruction),
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
        checksum = now.year + now.month + now.day + now.hour + now.minute + now.second
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

    # ---------------- Helpers ----------------
    def nm_to_kgfcm(self, nm):
        return round(nm * NM_TO_KGFCM, 2)

    def publish_string(self, publisher, text, log=True):
        msg = String()
        msg.data = text
        publisher.publish(msg)
        if log:
            self.get_logger().info(text)

    def publish_float(self, publisher, value):
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)

    def publish_human(self, text):
        self.publish_string(self.human_status_pub, text)

    def publish_event(self, text):
        self.publish_string(self.events_pub, text)

    def clean_text(self, text):
        return text.strip().strip("}").replace("_", "")

    def to_float(self, text):
        return float(text.strip().strip("}"))

    def decode_operation_mode(self, code):
        return {"0": "ADV", "1": "STD", "2": "ALI", "3": "SET"}.get(code, f"unknown ({code})")

    def decode_connected(self, code):
        return "connected" if code == "1" else "not connected"

    def decode_yes_no(self, code):
        return "yes" if code == "1" else "no"


def main(args=None):
    rclpy.init(args=args)
    node = KLTcgDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
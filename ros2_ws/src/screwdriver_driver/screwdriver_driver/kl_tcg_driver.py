import datetime
import threading
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

import serial


NM_TO_KGFCM = 10.19716213


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


OPERATION_MODES = {
    "0": "ADV",
    "1": "STD",
    "2": "ALI",
    "3": "SET",
}


class KLTcgDriver(Node):
    """ROS 2 driver for the KILEWS KL-TCG controller."""

    def __init__(self) -> None:
        super().__init__("kl_tcg_driver")

        # ============================================================
        # General parameters
        # ============================================================
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("device_name", 0)

        self.declare_parameter("target_job", 1)
        self.declare_parameter("target_sequence", 1)

        # ============================================================
        # Tightening Step parameters
        # ============================================================
        self.declare_parameter("step_number", 20)
        self.declare_parameter("step_name", "ROS020")
        self.declare_parameter("step_speed_rpm", 300)
        self.declare_parameter("step_direction", 0)
        self.declare_parameter("step_turns", 5.0)
        self.declare_parameter("step_torque_nm", 1.0)
        self.declare_parameter("step_delay_s", 0.8)

        # ============================================================
        # Tightening Program parameters
        # ============================================================
        self.declare_parameter("program_number", 20)
        self.declare_parameter("program_name", "ROS020")

        self.declare_parameter("program_step_1", 20)
        self.declare_parameter("program_step_2", 0)
        self.declare_parameter("program_step_3", 0)
        self.declare_parameter("program_step_4", 0)
        self.declare_parameter("program_step_5", 0)

        self.declare_parameter("program_okall_time_s", 1.0)
        self.declare_parameter("program_ok_time_s", 1.0)
        self.declare_parameter("program_okall_stop", 0)
        self.declare_parameter("program_ng_stop", 0)

        # ============================================================
        # Job parameters
        # ============================================================
        self.declare_parameter("job_number", 20)
        self.declare_parameter("job_sequence_1_program", 20)
        self.declare_parameter("job_sequence_1_repeat", 1)
        self.declare_parameter("job_reverse_direction", 0)
        self.declare_parameter("job_reverse_force", 5)
        self.declare_parameter("job_reverse_speed_rpm", 300)

        # ============================================================
        # CMD124 readback parameters
        # 1 = TS, 2 = TP, 3 = Job
        # ============================================================
        self.declare_parameter("read_select", 1)
        self.declare_parameter("read_number", 20)

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.device_name = int(
            self.get_parameter("device_name").value
        )

        # ============================================================
        # Publishers
        # ============================================================
        self.state_pub = self.create_publisher(
            String,
            "/screwdriver/state",
            10,
        )

        self.result_pub = self.create_publisher(
            String,
            "/screwdriver/result",
            10,
        )

        self.config_pub = self.create_publisher(
            String,
            "/screwdriver/config",
            10,
        )

        self.raw_frame_pub = self.create_publisher(
            String,
            "/screwdriver/raw_frame",
            10,
        )

        self.events_pub = self.create_publisher(
            String,
            "/screwdriver/events",
            10,
        )

        # ============================================================
        # Cached controller state
        # ============================================================
        self.current_controller_status = "connected"
        self.current_tool_status = "unknown"
        self.current_enabled = "unknown"
        self.current_mode = "unknown"

        self.current_job = "unknown"
        self.current_sequence = "unknown"
        self.current_program = "unknown"

        self.current_screw_count = "unknown"
        self.current_stop_code = "0"
        self.current_stop_text = "none"

        # ============================================================
        # Serial state
        # ============================================================
        self.ser: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()
        self._rx_buffer = ""
        self._instruction_number = 1
        self._last_cycle_count: Optional[int] = None

        # ============================================================
        # Services
        # ============================================================
        self.create_service(
            Trigger,
            "/screwdriver/read_controller_config",
            self.cb_read_controller_config,
        )

        self.create_service(
            Trigger,
            "/screwdriver/read_tool_status",
            self.cb_read_tool_status,
        )

        self.create_service(
            Trigger,
            "/screwdriver/execute_job",
            self.cb_execute_job,
        )

        self.create_service(
            Trigger,
            "/screwdriver/write_step",
            self.cb_write_step,
        )

        self.create_service(
            Trigger,
            "/screwdriver/write_program",
            self.cb_write_program,
        )

        self.create_service(
            Trigger,
            "/screwdriver/write_job",
            self.cb_write_job,
        )

        self.create_service(
            Trigger,
            "/screwdriver/read_setting",
            self.cb_read_setting,
        )

        # ============================================================
        # Serial connection
        # ============================================================
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=0.1,
            )

            self.publish_event(
                f"Connected to KL-TCG on {self.port} "
                f"at {self.baudrate} baud"
            )

        except serial.SerialException as exc:
            self.ser = None
            self.current_controller_status = "disconnected"

            self.publish_event(
                f"ERROR: Could not open serial port: {exc}"
            )

        self.timer = self.create_timer(
            0.02,
            self.read_serial,
        )

    # ================================================================
    # Serial reception
    # ================================================================
    def read_serial(self) -> None:
        if self.ser is None or not self.ser.is_open:
            return

        try:
            with self._serial_lock:
                available = self.ser.in_waiting

                if available <= 0:
                    return

                received = self.ser.read(available)

            self._rx_buffer += received.decode(
                "ascii",
                errors="ignore",
            )

        except serial.SerialException as exc:
            self.publish_event(
                f"ERROR: Serial read error: {exc}"
            )
            return

        self.extract_frames_from_buffer()

    def extract_frames_from_buffer(self) -> None:
        while True:
            start = self._rx_buffer.find("{")

            if start == -1:
                self._rx_buffer = ""
                return

            end = self._rx_buffer.find("}", start)

            if end == -1:
                self._rx_buffer = self._rx_buffer[start:]
                return

            frame = self._rx_buffer[start:end + 1]
            self._rx_buffer = self._rx_buffer[end + 1:]

            self.process_frame(frame)

    def process_frame(self, frame: str) -> None:
        self.publish_string(
            self.raw_frame_pub,
            frame,
            log=False,
        )

        fields = self.parse_frame(frame)

        if not fields:
            return

        command = fields[0]

        if command == "REQ100":
            self.handle_req100(fields, frame)

        elif command == "DATA100":
            self.handle_data100(fields, frame)

        elif command == "DATA101":
            pass

        elif command == "ANS104":
            self.handle_ans104(fields, frame)

        elif command == "ANS121":
            self.handle_ans121(fields, frame)

        elif command == "ANS122":
            self.handle_ans122(fields, frame)

        elif command == "ANS123":
            self.handle_ans123(fields, frame)

        elif command == "ANS124":
            self.handle_ans124(fields, frame)

        elif command.startswith("ANS"):
            self.publish_event(
                f"Controller answer: {command} | raw={frame}"
            )

        else:
            self.publish_event(
                f"Unknown frame: command={command}, raw={frame}"
            )

    @staticmethod
    def parse_frame(frame: str) -> List[str]:
        clean = frame.strip().strip("{}")

        if not clean:
            return []

        return clean.split(",")

    # ================================================================
    # REQ100
    # ================================================================
    def handle_req100(
        self,
        fields: List[str],
        raw_frame: str,
    ) -> None:
        try:
            operation_mode = fields[14]
            job = fields[16]
            sequence = fields[17]
            program = fields[19]
            tool_connected = fields[21]
            tool_enabled = fields[24]
            stop_code = fields[25]
            screw_count = fields[26]

            self.current_controller_status = "connected"
            self.current_tool_status = self.decode_connected(
                tool_connected
            )
            self.current_enabled = self.decode_yes_no(
                tool_enabled
            )
            self.current_mode = self.decode_operation_mode(
                operation_mode
            )

            self.current_job = job
            self.current_sequence = sequence
            self.current_program = program
            self.current_screw_count = screw_count

            self.current_stop_code = stop_code
            self.current_stop_text = TOOL_STOP_STATUS.get(
                stop_code,
                "unknown",
            )

            self.publish_current_state()

        except IndexError:
            self.publish_event(
                f"Incomplete REQ100 frame: {raw_frame}"
            )

    # ================================================================
    # DATA100
    # ================================================================
    def handle_data100(
        self,
        fields: List[str],
        raw_frame: str,
    ) -> None:
        try:
            cycle_count = int(fields[13])
            job = fields[14]
            sequence = fields[15]
            program = fields[16]

            torque = self.to_float(fields[19])
            torque_unit = TORQUE_UNITS.get(
                fields[20],
                "unknown",
            )

            fastening_time = self.to_float(fields[21])
            rotations = self.to_float(fields[22])
            screw_count = fields[23]
            status = self.clean_text(fields[25])

            stop_code = fields[26].strip("}")
            stop_text = TOOL_STOP_STATUS.get(
                stop_code,
                "unknown",
            )

            self.current_controller_status = "connected"
            self.current_tool_status = "connected"
            self.current_job = job
            self.current_sequence = sequence
            self.current_program = program
            self.current_screw_count = screw_count
            self.current_stop_code = stop_code
            self.current_stop_text = stop_text

            self.publish_current_state()

            if cycle_count != self._last_cycle_count:
                result = (
                    "LAST RESULT\n"
                    "-----------\n"
                    f"Result     : {status}\n"
                    f"Torque     : {torque:.3f} {torque_unit}\n"
                    f"Time       : {fastening_time:.3f} s\n"
                    f"Rotations  : {rotations:.1f}\n"
                    f"Cycle Nr.  : {cycle_count}"
                )

                self.publish_string(
                    self.result_pub,
                    result,
                )

                self._last_cycle_count = cycle_count

            self.send_cmd100_ack()

        except (IndexError, ValueError):
            self.publish_event(
                f"Could not decode DATA100 frame: {raw_frame}"
            )

    # ================================================================
    # ANS104 Job selection confirmation
    # ================================================================
    def handle_ans104(
        self,
        fields: List[str],
        raw_frame: str,
    ) -> None:
        try:
            status = self.get_answer_status(fields)

            message = self.build_status_message(
                title="JOB SELECTION RESULT",
                status=status,
            )

            self.publish_string(
                self.config_pub,
                message,
            )

            self.publish_event(
                f"Controller answer: ANS104 | raw={raw_frame}"
            )

        except (IndexError, ValueError):
            self.publish_event(
                f"Could not decode ANS104: {raw_frame}"
            )

    # ================================================================
    # ANS121 Step write confirmation
    # ================================================================
    def handle_ans121(
        self,
        fields: List[str],
        raw_frame: str,
    ) -> None:
        try:
            status = self.get_answer_status(fields)

            message = self.build_status_message(
                title="WRITE STEP RESULT",
                status=status,
            )

            self.publish_string(
                self.config_pub,
                message,
            )

            self.publish_event(
                f"Controller answer: ANS121 | raw={raw_frame}"
            )

        except (IndexError, ValueError):
            self.publish_event(
                f"Could not decode ANS121: {raw_frame}"
            )

    # ================================================================
    # ANS122 Program write confirmation
    # ================================================================
    def handle_ans122(
        self,
        fields: List[str],
        raw_frame: str,
    ) -> None:
        try:
            status = self.get_answer_status(fields)

            message = self.build_status_message(
                title="WRITE PROGRAM RESULT",
                status=status,
            )

            self.publish_string(
                self.config_pub,
                message,
            )

            self.publish_event(
                f"Controller answer: ANS122 | raw={raw_frame}"
            )

        except (IndexError, ValueError):
            self.publish_event(
                f"Could not decode ANS122: {raw_frame}"
            )

    # ================================================================
    # ANS123 Job write confirmation
    # ================================================================
    def handle_ans123(
        self,
        fields: List[str],
        raw_frame: str,
    ) -> None:
        try:
            status = self.get_answer_status(fields)

            message = self.build_status_message(
                title="WRITE JOB RESULT",
                status=status,
            )

            self.publish_string(
                self.config_pub,
                message,
            )

            self.publish_event(
                f"Controller answer: ANS123 | raw={raw_frame}"
            )

        except (IndexError, ValueError):
            self.publish_event(
                f"Could not decode ANS123: {raw_frame}"
            )

    # ================================================================
    # ANS124 TS / TP / Job readback
    # ================================================================
    def handle_ans124(
        self,
        fields: List[str],
        raw_frame: str,
    ) -> None:
        try:
            kind = fields[9]

            if kind == "1":
                torque_kgfcm = self.to_float(fields[16])
                torque_nm = torque_kgfcm / NM_TO_KGFCM

                upper_torque_kgfcm = self.to_float(
                    fields[21]
                )
                upper_torque_nm = (
                    upper_torque_kgfcm / NM_TO_KGFCM
                )

                block = (
                    "TIGHTENING STEP READBACK\n"
                    "------------------------\n"
                    f"Step number      : {fields[10]}\n"
                    f"Step name        : {fields[11]}\n"
                    f"Speed            : {fields[12]} rpm\n"
                    f"Target condition : "
                    f"{self.decode_target_condition(fields[13])}\n"
                    f"Target turns     : {fields[15]}\n"
                    f"Target torque    : "
                    f"{torque_nm:.3f} N.m "
                    f"({torque_kgfcm:.2f} kgf.cm)\n"
                    f"Direction        : "
                    f"{self.decode_direction(fields[17])}\n"
                    f"Delay            : {fields[18]} s\n"
                    f"Upper turns      : {fields[19]}\n"
                    f"Lower turns      : {fields[20]}\n"
                    f"Upper torque     : "
                    f"{upper_torque_nm:.3f} N.m\n"
                    f"Lower torque     : "
                    f"{fields[22]} kgf.cm"
                )

            elif kind == "2":
                block = (
                    "TIGHTENING PROGRAM READBACK\n"
                    "---------------------------\n"
                    f"Program number : {fields[10]}\n"
                    f"Program name   : {fields[11]}\n"
                    f"Step 1         : {fields[13]}\n"
                    f"Step 2         : {fields[14]}\n"
                    f"Step 3         : {fields[15]}\n"
                    f"Step 4         : {fields[16]}\n"
                    f"Step 5         : {fields[17]}"
                )

            elif kind == "3":
                block = (
                    "JOB READBACK\n"
                    "------------\n"
                    f"Job number : {fields[10]}\n"
                    f"Total JS   : {fields[11]}\n"
                    f"Sequence   : {fields[12]}\n"
                    f"Program    : {fields[13]}\n"
                    f"Repeat     : {fields[14]}"
                )

            else:
                block = (
                    "PARAMETER READBACK\n"
                    "------------------\n"
                    f"Unknown parameter type: {kind}"
                )

            self.publish_string(
                self.config_pub,
                block,
            )

            self.publish_event(
                f"Controller answer: ANS124 | raw={raw_frame}"
            )

        except (IndexError, ValueError):
            self.publish_event(
                f"Could not decode ANS124: {raw_frame}"
            )

    # ================================================================
    # State publishing
    # ================================================================
    def publish_current_state(self) -> None:
        state = (
            "STATE\n"
            "-----\n"
            f"Controller : {self.current_controller_status}\n"
            f"Tool       : {self.current_tool_status}\n"
            f"Enabled    : {self.current_enabled}\n"
            f"Mode       : {self.current_mode}\n"
            f"Job/Seq/TP : "
            f"{self.current_job}/"
            f"{self.current_sequence}/"
            f"{self.current_program}\n"
            f"Screws     : {self.current_screw_count}\n"
            f"Stop       : "
            f"{self.current_stop_code} "
            f"({self.current_stop_text})"
        )

        self.publish_string(
            self.state_pub,
            state,
            log=False,
        )

    # ================================================================
    # Service callbacks
    # ================================================================
    def cb_read_controller_config(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        frame = self.build_command_frame(108)
        success, information = self.send_frame(frame)

        response.success = success
        response.message = (
            f"CMD108 sent: {information}"
            if success
            else information
        )

        return response

    def cb_read_tool_status(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        frame = self.build_command_frame(116)
        success, information = self.send_frame(frame)

        response.success = success
        response.message = (
            f"CMD116 sent: {information}"
            if success
            else information
        )

        return response

    def cb_execute_job(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        job = int(
            self.get_parameter("target_job").value
        )
        sequence = int(
            self.get_parameter("target_sequence").value
        )

        if not 1 <= job <= 50:
            response.success = False
            response.message = (
                f"Invalid job={job}. Must be 1..50."
            )
            return response

        if not 1 <= sequence <= 50:
            response.success = False
            response.message = (
                f"Invalid sequence={sequence}. Must be 1..50."
            )
            return response

        frame = self.build_command_frame(
            104,
            extra_fields=[
                f"{job:02d}",
                f"{sequence:02d}",
            ],
        )

        success, information = self.send_frame(frame)

        response.success = success
        response.message = (
            f"CMD104 sent for Job={job}, "
            f"Sequence={sequence}: {information}"
            if success
            else information
        )

        return response

    def cb_read_setting(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        select = int(
            self.get_parameter("read_select").value
        )
        number = int(
            self.get_parameter("read_number").value
        )

        if select not in (1, 2, 3):
            response.success = False
            response.message = (
                "read_select must be "
                "1 (TS), 2 (TP), or 3 (Job)."
            )
            return response

        if number < 1:
            response.success = False
            response.message = (
                "read_number must be at least 1."
            )
            return response

        frame = self.build_command_frame(
            124,
            extra_fields=[
                str(select),
                str(number),
            ],
        )

        success, information = self.send_frame(frame)

        response.success = success
        response.message = (
            f"CMD124 sent: type={select}, "
            f"number={number}: {information}"
            if success
            else information
        )

        return response

    def cb_write_step(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        step = int(
            self.get_parameter("step_number").value
        )
        name = str(
            self.get_parameter("step_name").value
        ).upper()
        speed = int(
            self.get_parameter("step_speed_rpm").value
        )
        direction = int(
            self.get_parameter("step_direction").value
        )
        turns = float(
            self.get_parameter("step_turns").value
        )
        torque_nm = float(
            self.get_parameter("step_torque_nm").value
        )
        delay_s = float(
            self.get_parameter("step_delay_s").value
        )

        validation_error = self.validate_step_parameters(
            step=step,
            name=name,
            speed=speed,
            direction=direction,
            turns=turns,
            torque_nm=torque_nm,
            delay_s=delay_s,
        )

        if validation_error is not None:
            response.success = False
            response.message = validation_error
            return response

        torque_kgfcm = torque_nm * NM_TO_KGFCM

        extra_fields = [
            str(step),
            name,
            str(speed),
            "1",
            "0.000",
            f"{turns:.1f}",
            f"{torque_kgfcm:.2f}",
            str(direction),
            f"{delay_s:.1f}",
            "999.9",
            "0.0",
            f"{torque_kgfcm:.2f}",
            "0.00",
            "+0",
            "10",
            "10",
            "0",
            "+0",
        ]

        frame = self.build_command_frame(
            121,
            extra_fields=extra_fields,
        )

        success, information = self.send_frame(frame)

        response.success = success
        response.message = (
            f"CMD121 sent for Step={step}, "
            f"Speed={speed} rpm, "
            f"Direction={self.decode_direction(str(direction))}, "
            f"Turns={turns:.1f}, "
            f"Torque limit={torque_nm:.3f} N.m: "
            f"{information}"
            if success
            else information
        )

        return response

    def cb_write_program(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        program = int(
            self.get_parameter("program_number").value
        )
        name = str(
            self.get_parameter("program_name").value
        ).upper()

        steps = [
            int(self.get_parameter("program_step_1").value),
            int(self.get_parameter("program_step_2").value),
            int(self.get_parameter("program_step_3").value),
            int(self.get_parameter("program_step_4").value),
            int(self.get_parameter("program_step_5").value),
        ]

        okall_time = float(
            self.get_parameter("program_okall_time_s").value
        )
        ok_time = float(
            self.get_parameter("program_ok_time_s").value
        )
        okall_stop = int(
            self.get_parameter("program_okall_stop").value
        )
        ng_stop = int(
            self.get_parameter("program_ng_stop").value
        )

        validation_error = self.validate_program_parameters(
            program=program,
            name=name,
            steps=steps,
            okall_time=okall_time,
            ok_time=ok_time,
            okall_stop=okall_stop,
            ng_stop=ng_stop,
        )

        if validation_error is not None:
            response.success = False
            response.message = validation_error
            return response

        extra_fields = [
            str(program),
            name,
            "0",
            str(steps[0]),
            str(steps[1]),
            str(steps[2]),
            str(steps[3]),
            str(steps[4]),
            f"{okall_time:.1f}",
            f"{ok_time:.1f}",
            str(okall_stop),
            str(ng_stop),
            "0",
            "0",
            "0",
        ]

        frame = self.build_command_frame(
            122,
            extra_fields=extra_fields,
        )

        success, information = self.send_frame(frame)

        response.success = success
        response.message = (
            f"CMD122 sent for TP={program}, "
            f"Steps={steps}: {information}"
            if success
            else information
        )

        return response

    def cb_write_job(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        job_number = int(
            self.get_parameter("job_number").value
        )
        program = int(
            self.get_parameter(
                "job_sequence_1_program"
            ).value
        )
        repeat = int(
            self.get_parameter(
                "job_sequence_1_repeat"
            ).value
        )
        reverse_direction = int(
            self.get_parameter(
                "job_reverse_direction"
            ).value
        )
        reverse_force = int(
            self.get_parameter(
                "job_reverse_force"
            ).value
        )
        reverse_speed = int(
            self.get_parameter(
                "job_reverse_speed_rpm"
            ).value
        )

        validation_error = self.validate_job_parameters(
            job_number=job_number,
            program=program,
            repeat=repeat,
            reverse_direction=reverse_direction,
            reverse_force=reverse_force,
            reverse_speed=reverse_speed,
        )

        if validation_error is not None:
            response.success = False
            response.message = validation_error
            return response

        sequence_fields = [
            str(program),
            str(repeat),
        ]

        for _ in range(24):
            sequence_fields.extend(["0", "0"])

        extra_fields = [
            str(job_number),
            "0",
            *sequence_fields,
            str(reverse_direction),
            str(reverse_force),
            str(reverse_speed),
        ]

        frame = self.build_command_frame(
            123,
            extra_fields=extra_fields,
        )

        success, information = self.send_frame(frame)

        response.success = success
        response.message = (
            f"CMD123 sent for Job={job_number}, "
            f"Sequence 1 -> TP={program}, "
            f"Repeat={repeat}: {information}"
            if success
            else information
        )

        return response

    # ================================================================
    # Validation
    # ================================================================
    @staticmethod
    def validate_step_parameters(
        step: int,
        name: str,
        speed: int,
        direction: int,
        turns: float,
        torque_nm: float,
        delay_s: float,
    ) -> Optional[str]:
        if not 1 <= step <= 250:
            return "step_number must be between 1 and 250."

        if not 1 <= len(name) <= 6:
            return (
                "step_name must contain between "
                "1 and 6 characters."
            )

        if not name.replace("_", "").isalnum():
            return (
                "step_name may only contain letters, "
                "numbers, or underscores."
            )

        if not 160 <= speed <= 1000:
            return (
                "step_speed_rpm must be between "
                "160 and 1000."
            )

        if direction not in (0, 1):
            return (
                "step_direction must be "
                "0 (CCW) or 1 (CW)."
            )

        if not 0.1 <= turns <= 50.0:
            return (
                "step_turns must be between "
                "0.1 and 50.0."
            )

        if not 0.6 <= torque_nm <= 3.0:
            return (
                "step_torque_nm must be between "
                "0.6 and 3.0 N.m."
            )

        if not 0.0 <= delay_s <= 9.9:
            return (
                "step_delay_s must be between "
                "0.0 and 9.9."
            )

        torque_kgfcm = torque_nm * NM_TO_KGFCM

        if torque_kgfcm > 99.0:
            return (
                "Converted torque exceeds "
                "99.00 kgf.cm."
            )

        return None

    @staticmethod
    def validate_program_parameters(
        program: int,
        name: str,
        steps: List[int],
        okall_time: float,
        ok_time: float,
        okall_stop: int,
        ng_stop: int,
    ) -> Optional[str]:
        if not 1 <= program <= 99:
            return (
                "program_number must be between "
                "1 and 99."
            )

        if not 1 <= len(name) <= 6:
            return (
                "program_name must contain between "
                "1 and 6 characters."
            )

        if not name.replace("_", "").isalnum():
            return (
                "program_name may only contain letters, "
                "numbers, or underscores."
            )

        if not 1 <= steps[0] <= 250:
            return (
                "program_step_1 must reference "
                "a TS between 1 and 250."
            )

        for index, step in enumerate(
            steps[1:],
            start=2,
        ):
            if not 0 <= step <= 250:
                return (
                    f"program_step_{index} must be "
                    "between 0 and 250."
                )

        if not 0.0 <= okall_time <= 9.9:
            return (
                "program_okall_time_s must be "
                "between 0.0 and 9.9."
            )

        if not 0.0 <= ok_time <= 9.9:
            return (
                "program_ok_time_s must be "
                "between 0.0 and 9.9."
            )

        if okall_stop not in (0, 1):
            return (
                "program_okall_stop must be "
                "0 or 1."
            )

        if not 0 <= ng_stop <= 9:
            return (
                "program_ng_stop must be "
                "between 0 and 9."
            )

        return None

    @staticmethod
    def validate_job_parameters(
        job_number: int,
        program: int,
        repeat: int,
        reverse_direction: int,
        reverse_force: int,
        reverse_speed: int,
    ) -> Optional[str]:
        if not 1 <= job_number <= 50:
            return (
                "job_number must be between "
                "1 and 50."
            )

        if not 1 <= program <= 99:
            return (
                "job_sequence_1_program must be "
                "between 1 and 99."
            )

        if not 1 <= repeat <= 99:
            return (
                "job_sequence_1_repeat must be "
                "between 1 and 99."
            )

        if reverse_direction not in (0, 1):
            return (
                "job_reverse_direction must be "
                "0 or 1."
            )

        if not 0 <= reverse_force <= 9:
            return (
                "job_reverse_force must be "
                "between 0 and 9."
            )

        if not 160 <= reverse_speed <= 1000:
            return (
                "job_reverse_speed_rpm must be "
                "between 160 and 1000."
            )

        return None

    # ================================================================
    # Command transmission
    # ================================================================
    def send_cmd100_ack(self) -> None:
        frame = self.build_command_frame(100)
        success, information = self.send_frame(frame)

        if not success:
            self.publish_event(
                f"CMD100 acknowledgement failed: "
                f"{information}"
            )

    def build_command_frame(
        self,
        command_code: int,
        extra_fields: Optional[List[str]] = None,
    ) -> str:
        if extra_fields is None:
            extra_fields = []

        timestamp = self.time_fields()
        instruction = self.next_instruction_number()

        fields = [
            f"CMD{command_code}",
            timestamp["year"],
            timestamp["month"],
            timestamp["day"],
            timestamp["hour"],
            timestamp["minute"],
            timestamp["second"],
            timestamp["checksum"],
            timestamp["keycode"],
            str(self.device_name),
            *extra_fields,
            str(instruction),
        ]

        return "{" + ",".join(fields) + ",}"

    def send_frame(
        self,
        frame: str,
    ) -> Tuple[bool, str]:
        if self.ser is None or not self.ser.is_open:
            return False, "serial port not connected"

        try:
            data = (frame + "\r\n").encode("ascii")

            with self._serial_lock:
                self.ser.write(data)
                self.ser.flush()

            self.publish_event(
                f"Sent command: {frame}"
            )

            return True, frame

        except serial.SerialException as exc:
            return False, f"serial write error: {exc}"

    # ================================================================
    # Helpers
    # ================================================================
    @staticmethod
    def get_answer_status(
        fields: List[str],
    ) -> str:
        useful_fields = [
            value.strip()
            for value in fields
            if value.strip() != ""
        ]

        if not useful_fields:
            raise ValueError(
                "Answer frame contains no usable fields."
            )

        return useful_fields[-1]

    @staticmethod
    def build_status_message(
        title: str,
        status: str,
    ) -> str:
        separator = "-" * len(title)

        if status == "1":
            status_text = "accepted by controller"
        elif status == "0":
            status_text = "rejected by controller"
        else:
            status_text = f"unknown ({status})"

        return (
            f"{title}\n"
            f"{separator}\n"
            f"Status : {status_text}"
        )

    def next_instruction_number(self) -> int:
        number = self._instruction_number
        self._instruction_number += 1

        if self._instruction_number > 255:
            self._instruction_number = 1

        return number

    @staticmethod
    def time_fields() -> dict:
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

    def publish_string(
        self,
        publisher,
        text: str,
        log: bool = True,
    ) -> None:
        message = String()
        message.data = text
        publisher.publish(message)

        if log:
            self.get_logger().info(text)

    def publish_event(self, text: str) -> None:
        self.publish_string(
            self.events_pub,
            text,
        )

    @staticmethod
    def clean_text(text: str) -> str:
        return (
            text.strip()
            .strip("}")
            .replace("_", "")
        )

    @staticmethod
    def to_float(text: str) -> float:
        return float(
            text.strip().strip("}")
        )

    @staticmethod
    def decode_operation_mode(code: str) -> str:
        return OPERATION_MODES.get(
            code,
            f"unknown ({code})",
        )

    @staticmethod
    def decode_connected(code: str) -> str:
        return (
            "connected"
            if code == "1"
            else "not connected"
        )

    @staticmethod
    def decode_yes_no(code: str) -> str:
        return "yes" if code == "1" else "no"

    @staticmethod
    def decode_direction(code: str) -> str:
        return {
            "0": "CCW / reverse",
            "1": "CW / forward",
        }.get(code, f"unknown ({code})")

    @staticmethod
    def decode_target_condition(code: str) -> str:
        return {
            "1": "Thread / turns",
            "2": "Torque",
        }.get(code, f"unknown ({code})")

    def destroy_node(self) -> None:
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
            except serial.SerialException:
                pass

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KLTcgDriver()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger
import serial
import datetime
import threading


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
        # DEVICE_NAME du protocole (0: AMS, 1: DAS).
        # A verifier: doit correspondre a la valeur de ton CMD100 fonctionnel.
        self.declare_parameter("device_name", 0)

        self.device_name = int(self.get_parameter("device_name").value)

        # Numero d'instruction: on l'incremente a chaque commande envoyee.
        # Le protocole demande 1~255.
        self._instruction_number = 1
        # Verrou pour ne pas melanger ecriture serie et boucle de lecture.
        self._serial_lock = threading.Lock()

        # --- Publishers (lecture, inchanges par rapport a ta version) ---
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

        # --- Services de pilotage ---
        # CMD108: lecture config controleur (NON destructif, a tester en premier)
        self.srv_read_config = self.create_service(
            Trigger, "/screwdriver/read_controller_config", self.cb_read_config
        )
        # CMD116: lecture statut visseuse (NON destructif)
        self.srv_read_tool = self.create_service(
            Trigger, "/screwdriver/read_tool_status", self.cb_read_tool_status
        )
        # CMD104: choix/execution d'un job (POTENTIELLEMENT declenchant)
        # Trigger n'a pas de champ d'entree, donc job/sequence viennent de parametres
        # ROS que tu fixes avant l'appel. Simple et sur pour un test.
        self.declare_parameter("target_job", 1)
        self.declare_parameter("target_sequence", 1)
        self.srv_execute_job = self.create_service(
            Trigger, "/screwdriver/execute_job", self.cb_execute_job
        )

        self.get_logger().info("Piloting services ready: "
                               "read_controller_config, read_tool_status, execute_job")

    # ------------------------------------------------------------------
    # Construction de trame
    # ------------------------------------------------------------------
    def _next_instruction_number(self):
        n = self._instruction_number
        self._instruction_number += 1
        if self._instruction_number > 255:
            self._instruction_number = 1
        return n

    def _time_fields(self):
        now = datetime.datetime.now()
        year = now.year
        month = now.month
        day = now.day
        hour = now.hour
        minute = now.minute
        second = now.second
        checksum = year + month + day + hour + minute + second
        keycode = checksum + 5438
        # Format: annee 4 chiffres, reste 2 chiffres, checksum/keycode 4 chiffres.
        # (conforme a ta capture reelle CMD100 qui utilise 4 caracteres)
        return {
            "year": f"{year:04d}",
            "month": f"{month:02d}",
            "day": f"{day:02d}",
            "hour": f"{hour:02d}",
            "minute": f"{minute:02d}",
            "second": f"{second:02d}",
            "checksum": f"{checksum % 10000:04d}",
            "keycode": f"{keycode % 10000:04d}",
        }

    def _build_frame(self, cmd_code, extra_fields=None):
        """Construit une trame CMD1xx. extra_fields = liste de str inseres
        entre device_name et instruction_number."""
        t = self._time_fields()
        instr = self._next_instruction_number()
        parts = [
            f"{{CMD{cmd_code}",
            t["year"], t["month"], t["day"],
            t["hour"], t["minute"], t["second"],
            t["checksum"], t["keycode"],
            str(self.device_name),
        ]
        if extra_fields:
            parts.extend(str(f) for f in extra_fields)
        parts.append(str(instr))
        # Le protocole se termine par une accolade fermante.
        frame = ",".join(parts) + ",}"
        return frame

    def _send_frame(self, frame):
        if self.ser is None:
            return False, "serial port not open"
        try:
            # ASCII + LF + CR d'apres le protocole.
            payload = (frame + "\r\n").encode("ascii")
            with self._serial_lock:
                self.ser.write(payload)
            self.get_logger().info(f"Sent frame: {frame}")
            return True, frame
        except Exception as e:
            self.get_logger().error(f"Send error: {e}")
            return False, str(e)

    # ------------------------------------------------------------------
    # Callbacks des services
    # ------------------------------------------------------------------
    def cb_read_config(self, request, response):
        # CMD108: pas de champ specifique, juste lecture.
        frame = self._build_frame(108)
        ok, info = self._send_frame(frame)
        response.success = ok
        response.message = (f"CMD108 sent ({info}). "
                            f"Watch /screwdriver/raw_data for an ANS108 reply.") if ok else info
        return response

    def cb_read_tool_status(self, request, response):
        # CMD116: lecture statut visseuse.
        frame = self._build_frame(116)
        ok, info = self._send_frame(frame)
        response.success = ok
        response.message = (f"CMD116 sent ({info}). "
                            f"Watch /screwdriver/raw_data for an ANS116 reply.") if ok else info
        return response

    def cb_execute_job(self, request, response):
        # CMD104: str11 = Job, str12 = Job Sequence.
        job = int(self.get_parameter("target_job").value)
        seq = int(self.get_parameter("target_sequence").value)
        if not (1 <= job <= 50) or not (1 <= seq <= 50):
            response.success = False
            response.message = f"Job/Sequence out of range (job={job}, seq={seq}, must be 1..50)"
            return response
        frame = self._build_frame(104, extra_fields=[f"{job:02d}", f"{seq:02d}"])
        ok, info = self._send_frame(frame)
        response.success = ok
        response.message = (f"CMD104 sent for job={job}, seq={seq} ({info}). "
                            f"Watch ANS104 and observe whether the tool rotates.") if ok else info
        return response

    # ------------------------------------------------------------------
    # Lecture serie (ta logique, + reconnaissance des ANS)
    # ------------------------------------------------------------------
    def read_serial(self):
        if self.ser is None:
            return
        try:
            with self._serial_lock:
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
            elif command.startswith("ANS"):
                # Reponses aux commandes de pilotage (ANS104/108/116...).
                self.publish_string(
                    self.human_status_pub,
                    f"Reply to piloting command ({command}): {line}"
                )
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
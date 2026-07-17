import tkinter as tk
from tkinter import messagebox, ttk
from typing import Callable, Optional

import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


DRIVER_NODE = "/kl_tcg_driver"

FIXED_JOB = 1
FIXED_SEQUENCE = 1
FIXED_PROGRAM = 1
FIXED_STEP = 1


class ScrewdriverGui(Node):
    """Simple GUI for the fixed Job 1 -> TP 1 -> TS 1 structure."""

    def __init__(self) -> None:
        super().__init__("screwdriver_gui")

        self.root = tk.Tk()
        self.root.title("KL-TCG Screwdriver Control")
        self.root.geometry("900x650")
        self.root.protocol("WM_DELETE_WINDOW", self.close_application)

        self.parameter_client = self.create_client(
            SetParameters,
            f"{DRIVER_NODE}/set_parameters",
        )

        self.write_step_client = self.create_client(
            Trigger,
            "/screwdriver/write_step",
        )

        self.write_program_client = self.create_client(
            Trigger,
            "/screwdriver/write_program",
        )

        self.write_job_client = self.create_client(
            Trigger,
            "/screwdriver/write_job",
        )

        self.read_setting_client = self.create_client(
            Trigger,
            "/screwdriver/read_setting",
        )

        self.execute_job_client = self.create_client(
            Trigger,
            "/screwdriver/execute_job",
        )

        self.create_subscription(
            String,
            "/screwdriver/state",
            self.state_callback,
            10,
        )

        self.create_subscription(
            String,
            "/screwdriver/result",
            self.result_callback,
            10,
        )

        self.create_subscription(
            String,
            "/screwdriver/config",
            self.config_callback,
            10,
        )

        self.speed_var = tk.IntVar(value=300)
        self.direction_var = tk.StringVar(value="CCW / Unscrewing")
        self.turns_var = tk.DoubleVar(value=5.0)
        self.torque_var = tk.DoubleVar(value=1.0)
        self.delay_var = tk.DoubleVar(value=0.8)

        self.status_var = tk.StringVar(value="Waiting for driver...")
        self.state_text: Optional[tk.Text] = None
        self.result_text: Optional[tk.Text] = None
        self.config_text: Optional[tk.Text] = None

        self.create_interface()

        # Integrate ROS processing into the Tkinter event loop.
        self.root.after(50, self.process_ros_events)

    # ================================================================
    # Interface
    # ================================================================
    def create_interface(self) -> None:
        main = ttk.Frame(
            self.root,
            padding=16,
        )
        main.pack(
            fill=tk.BOTH,
            expand=True,
        )

        title = ttk.Label(
            main,
            text="KL-TCG Screwdriver Control",
            font=("Arial", 18, "bold"),
        )
        title.pack(pady=(0, 12))

        structure_label = ttk.Label(
            main,
            text="Fixed configuration: Job 1  →  Sequence 1  →  TP 1  →  TS 1",
        )
        structure_label.pack(pady=(0, 16))

        configuration_frame = ttk.LabelFrame(
            main,
            text="Unscrewing parameters",
            padding=12,
        )
        configuration_frame.pack(
            fill=tk.X,
            pady=(0, 12),
        )

        self.add_spinbox_row(
            configuration_frame,
            row=0,
            label="Speed",
            variable=self.speed_var,
            minimum=160,
            maximum=1000,
            increment=10,
            unit="rpm",
        )

        ttk.Label(
            configuration_frame,
            text="Direction",
        ).grid(
            row=1,
            column=0,
            sticky="w",
            padx=6,
            pady=6,
        )

        direction_box = ttk.Combobox(
            configuration_frame,
            textvariable=self.direction_var,
            values=[
                "CCW / Unscrewing",
                "CW / Tightening",
            ],
            state="readonly",
            width=22,
        )
        direction_box.grid(
            row=1,
            column=1,
            sticky="ew",
            padx=6,
            pady=6,
        )

        self.add_spinbox_row(
            configuration_frame,
            row=2,
            label="Target turns",
            variable=self.turns_var,
            minimum=0.1,
            maximum=50.0,
            increment=0.1,
            unit="turns",
        )

        self.add_spinbox_row(
            configuration_frame,
            row=3,
            label="Torque limit",
            variable=self.torque_var,
            minimum=0.6,
            maximum=3.0,
            increment=0.1,
            unit="N·m",
        )

        self.add_spinbox_row(
            configuration_frame,
            row=4,
            label="Delay",
            variable=self.delay_var,
            minimum=0.0,
            maximum=9.9,
            increment=0.1,
            unit="s",
        )

        configuration_frame.columnconfigure(
            1,
            weight=1,
        )

        button_frame = ttk.Frame(main)
        button_frame.pack(
            fill=tk.X,
            pady=(0, 12),
        )

        self.apply_button = ttk.Button(
            button_frame,
            text="Apply Configuration",
            command=self.apply_configuration,
        )
        self.apply_button.pack(
            side=tk.LEFT,
            expand=True,
            fill=tk.X,
            padx=(0, 6),
        )

        self.verify_button = ttk.Button(
            button_frame,
            text="Read Back TS 1",
            command=self.read_step,
        )
        self.verify_button.pack(
            side=tk.LEFT,
            expand=True,
            fill=tk.X,
            padx=6,
        )

        self.start_button = ttk.Button(
            button_frame,
            text="Start Cycle",
            command=self.start_cycle,
        )
        self.start_button.pack(
            side=tk.LEFT,
            expand=True,
            fill=tk.X,
            padx=(6, 0),
        )

        status_label = ttk.Label(
            main,
            textvariable=self.status_var,
            font=("Arial", 10, "bold"),
        )
        status_label.pack(
            fill=tk.X,
            pady=(0, 12),
        )

        monitor_frame = ttk.Frame(main)
        monitor_frame.pack(
            fill=tk.BOTH,
            expand=True,
        )

        self.state_text = self.create_monitor_box(
            monitor_frame,
            "Controller State",
            column=0,
        )

        self.result_text = self.create_monitor_box(
            monitor_frame,
            "Last Result",
            column=1,
        )

        self.config_text = self.create_monitor_box(
            monitor_frame,
            "Configuration Response",
            column=2,
        )

        for column in range(3):
            monitor_frame.columnconfigure(
                column,
                weight=1,
            )

        monitor_frame.rowconfigure(
            0,
            weight=1,
        )

    def add_spinbox_row(
        self,
        parent,
        row: int,
        label: str,
        variable,
        minimum: float,
        maximum: float,
        increment: float,
        unit: str,
    ) -> None:
        ttk.Label(
            parent,
            text=label,
        ).grid(
            row=row,
            column=0,
            sticky="w",
            padx=6,
            pady=6,
        )

        spinbox = ttk.Spinbox(
            parent,
            textvariable=variable,
            from_=minimum,
            to=maximum,
            increment=increment,
            width=15,
        )
        spinbox.grid(
            row=row,
            column=1,
            sticky="ew",
            padx=6,
            pady=6,
        )

        ttk.Label(
            parent,
            text=unit,
        ).grid(
            row=row,
            column=2,
            sticky="w",
            padx=6,
            pady=6,
        )

    def create_monitor_box(
        self,
        parent,
        title: str,
        column: int,
    ) -> tk.Text:
        frame = ttk.LabelFrame(
            parent,
            text=title,
            padding=6,
        )
        frame.grid(
            row=0,
            column=column,
            sticky="nsew",
            padx=4,
        )

        text = tk.Text(
            frame,
            wrap=tk.WORD,
            width=28,
            height=16,
            state=tk.DISABLED,
        )
        text.pack(
            fill=tk.BOTH,
            expand=True,
        )

        return text

    # ================================================================
    # Configuration workflow
    # ================================================================
    def apply_configuration(self) -> None:
        try:
            speed = int(self.speed_var.get())
            turns = float(self.turns_var.get())
            torque = float(self.torque_var.get())
            delay = float(self.delay_var.get())

        except (tk.TclError, ValueError):
            messagebox.showerror(
                "Invalid input",
                "At least one parameter contains an invalid value.",
            )
            return

        error = self.validate_values(
            speed=speed,
            turns=turns,
            torque=torque,
            delay=delay,
        )

        if error:
            messagebox.showerror(
                "Invalid configuration",
                error,
            )
            return

        direction = (
            0
            if self.direction_var.get().startswith("CCW")
            else 1
        )

        parameters = {
            # TS 1
            "step_number": FIXED_STEP,
            "step_name": "ROS001",
            "step_speed_rpm": speed,
            "step_direction": direction,
            "step_turns": turns,
            "step_torque_nm": torque,
            "step_delay_s": delay,

            # TP 1
            "program_number": FIXED_PROGRAM,
            "program_name": "ROS001",
            "program_step_1": FIXED_STEP,
            "program_step_2": 0,
            "program_step_3": 0,
            "program_step_4": 0,
            "program_step_5": 0,

            # Job 1
            "job_number": FIXED_JOB,
            "job_sequence_1_program": FIXED_PROGRAM,
            "job_sequence_1_repeat": 1,

            # CMD104 target
            "target_job": FIXED_JOB,
            "target_sequence": FIXED_SEQUENCE,
        }

        self.set_controls_enabled(False)
        self.status_var.set("Applying ROS parameters...")

        self.set_driver_parameters(
            parameters,
            callback=self.after_parameters_set,
        )

    def after_parameters_set(
        self,
        success: bool,
        message: str,
    ) -> None:
        if not success:
            self.finish_operation(
                False,
                f"Parameter update failed: {message}",
            )
            return

        self.status_var.set("Writing TS 1...")
        self.call_trigger_service(
            self.write_step_client,
            callback=self.after_step_written,
        )

    def after_step_written(
        self,
        success: bool,
        message: str,
    ) -> None:
        if not success:
            self.finish_operation(
                False,
                f"TS write failed: {message}",
            )
            return

        self.status_var.set("Writing TP 1...")
        self.call_trigger_service(
            self.write_program_client,
            callback=self.after_program_written,
        )

    def after_program_written(
        self,
        success: bool,
        message: str,
    ) -> None:
        if not success:
            self.finish_operation(
                False,
                f"TP write failed: {message}",
            )
            return

        self.status_var.set("Writing Job 1...")
        self.call_trigger_service(
            self.write_job_client,
            callback=self.after_job_written,
        )

    def after_job_written(
        self,
        success: bool,
        message: str,
    ) -> None:
        if not success:
            self.finish_operation(
                False,
                f"Job write failed: {message}",
            )
            return

        self.status_var.set(
            "Configuration commands sent. Reading TS 1 back..."
        )

        read_parameters = {
            "read_select": 1,
            "read_number": FIXED_STEP,
        }

        self.set_driver_parameters(
            read_parameters,
            callback=self.after_read_parameters_set,
        )

    def after_read_parameters_set(
        self,
        success: bool,
        message: str,
    ) -> None:
        if not success:
            self.finish_operation(
                False,
                f"Readback setup failed: {message}",
            )
            return

        self.call_trigger_service(
            self.read_setting_client,
            callback=self.after_configuration_readback,
        )

    def after_configuration_readback(
        self,
        success: bool,
        message: str,
    ) -> None:
        self.finish_operation(
            success,
            (
                "Configuration applied. Check the readback panel."
                if success
                else f"Readback failed: {message}"
            ),
        )

    # ================================================================
    # Manual readback
    # ================================================================
    def read_step(self) -> None:
        self.set_controls_enabled(False)
        self.status_var.set("Reading TS 1...")

        self.set_driver_parameters(
            {
                "read_select": 1,
                "read_number": FIXED_STEP,
            },
            callback=self.after_manual_read_parameters,
        )

    def after_manual_read_parameters(
        self,
        success: bool,
        message: str,
    ) -> None:
        if not success:
            self.finish_operation(
                False,
                message,
            )
            return

        self.call_trigger_service(
            self.read_setting_client,
            callback=self.after_manual_read,
        )

    def after_manual_read(
        self,
        success: bool,
        message: str,
    ) -> None:
        self.finish_operation(
            success,
            (
                "TS 1 readback requested."
                if success
                else message
            ),
        )

    # ================================================================
    # Cycle start
    # ================================================================
    def start_cycle(self) -> None:
        confirmed = messagebox.askyesno(
            "Start screwdriver cycle",
            (
                "The screwdriver may start immediately because "
                "CN1/CN2 are currently connected.\n\n"
                "Is the workspace safe?"
            ),
        )

        if not confirmed:
            return

        self.set_controls_enabled(False)
        self.status_var.set("Selecting and starting Job 1...")

        self.set_driver_parameters(
            {
                "target_job": FIXED_JOB,
                "target_sequence": FIXED_SEQUENCE,
            },
            callback=self.after_start_parameters_set,
        )

    def after_start_parameters_set(
        self,
        success: bool,
        message: str,
    ) -> None:
        if not success:
            self.finish_operation(
                False,
                message,
            )
            return

        self.call_trigger_service(
            self.execute_job_client,
            callback=self.after_cycle_started,
        )

    def after_cycle_started(
        self,
        success: bool,
        message: str,
    ) -> None:
        self.finish_operation(
            success,
            (
                "Job 1 command sent."
                if success
                else message
            ),
        )

    # ================================================================
    # ROS helpers
    # ================================================================
    def set_driver_parameters(
        self,
        parameters: dict,
        callback: Callable[[bool, str], None],
    ) -> None:
        if not self.parameter_client.service_is_ready():
            if not self.parameter_client.wait_for_service(
                timeout_sec=1.0
            ):
                callback(
                    False,
                    "Driver parameter service is unavailable.",
                )
                return

        request = SetParameters.Request()

        for name, value in parameters.items():
            request.parameters.append(
                self.create_parameter_message(
                    name,
                    value,
                )
            )

        future = self.parameter_client.call_async(request)

        def response_callback(completed_future) -> None:
            try:
                response = completed_future.result()

                failed_results = [
                    result.reason
                    for result in response.results
                    if not result.successful
                ]

                if failed_results:
                    callback(
                        False,
                        "; ".join(failed_results),
                    )
                else:
                    callback(
                        True,
                        "Parameters updated.",
                    )

            except Exception as exc:
                callback(
                    False,
                    str(exc),
                )

        future.add_done_callback(response_callback)

    @staticmethod
    def create_parameter_message(
        name: str,
        value,
    ) -> ParameterMsg:
        message = ParameterMsg()
        message.name = name

        parameter_value = ParameterValue()

        if isinstance(value, bool):
            parameter_value.type = ParameterType.PARAMETER_BOOL
            parameter_value.bool_value = value

        elif isinstance(value, int):
            parameter_value.type = ParameterType.PARAMETER_INTEGER
            parameter_value.integer_value = value

        elif isinstance(value, float):
            parameter_value.type = ParameterType.PARAMETER_DOUBLE
            parameter_value.double_value = value

        elif isinstance(value, str):
            parameter_value.type = ParameterType.PARAMETER_STRING
            parameter_value.string_value = value

        else:
            raise TypeError(
                f"Unsupported parameter type for {name}"
            )

        message.value = parameter_value
        return message

    def call_trigger_service(
        self,
        client,
        callback: Callable[[bool, str], None],
    ) -> None:
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=1.0):
                callback(
                    False,
                    "ROS service is unavailable.",
                )
                return

        future = client.call_async(Trigger.Request())

        def response_callback(completed_future) -> None:
            try:
                response = completed_future.result()
                callback(
                    response.success,
                    response.message,
                )

            except Exception as exc:
                callback(
                    False,
                    str(exc),
                )

        future.add_done_callback(response_callback)

    # ================================================================
    # Topic callbacks
    # ================================================================
    def state_callback(self, message: String) -> None:
        self.update_text_box(
            self.state_text,
            message.data,
        )

    def result_callback(self, message: String) -> None:
        self.update_text_box(
            self.result_text,
            message.data,
        )

    def config_callback(self, message: String) -> None:
        self.update_text_box(
            self.config_text,
            message.data,
        )

    @staticmethod
    def update_text_box(
        text_box: Optional[tk.Text],
        content: str,
    ) -> None:
        if text_box is None:
            return

        text_box.configure(state=tk.NORMAL)
        text_box.delete("1.0", tk.END)
        text_box.insert(tk.END, content)
        text_box.configure(state=tk.DISABLED)

    # ================================================================
    # General helpers
    # ================================================================
    @staticmethod
    def validate_values(
        speed: int,
        turns: float,
        torque: float,
        delay: float,
    ) -> Optional[str]:
        if not 160 <= speed <= 1000:
            return "Speed must be between 160 and 1000 rpm."

        if not 0.1 <= turns <= 50.0:
            return "Turns must be between 0.1 and 50.0."

        if not 0.6 <= torque <= 3.0:
            return "Torque must be between 0.6 and 3.0 N·m."

        if not 0.0 <= delay <= 9.9:
            return "Delay must be between 0.0 and 9.9 seconds."

        return None

    def finish_operation(
        self,
        success: bool,
        text: str,
    ) -> None:
        self.status_var.set(
            ("✓ " if success else "✗ ") + text
        )

        self.set_controls_enabled(True)

        if not success:
            messagebox.showerror(
                "Operation failed",
                text,
            )

    def set_controls_enabled(
        self,
        enabled: bool,
    ) -> None:
        state = (
            tk.NORMAL
            if enabled
            else tk.DISABLED
        )

        self.apply_button.configure(state=state)
        self.verify_button.configure(state=state)
        self.start_button.configure(state=state)

    def process_ros_events(self) -> None:
        if not rclpy.ok():
            return

        rclpy.spin_once(
            self,
            timeout_sec=0.0,
        )

        self.root.after(
            50,
            self.process_ros_events,
        )

    def run(self) -> None:
        self.root.mainloop()

    def close_application(self) -> None:
        self.root.destroy()

        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = ScrewdriverGui()

    try:
        node.run()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

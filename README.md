# FuE Project – KILEWS KL-TCG ROS 2 Driver

This package provides a ROS 2 interface for the KILEWS KL-TCG screwdriver controller.

The driver communicates with the controller through RS-232 and exposes controller monitoring, configuration and Job execution through ROS 2 topics, services and parameters.

---

# 1. Implemented Functions

The current driver supports:

- RS-232 communication at 115200 baud
- Buffered extraction of complete `{...}` telegrams
- Controller-state decoding from `REQ100`
- Final cycle-result decoding from `DATA100`
- Automatic `CMD100` acknowledgement of `DATA100`
- Job and Sequence selection with `CMD104`
- Controller-configuration request with `CMD108`
- Tool-status request with `CMD116`
- Tightening Step writing with `CMD121`
- Tightening Program writing with `CMD122`
- Job writing with `CMD123`
- TS, TP and Job read-back with `CMD124`
- Parameter validation before transmission
- Publication of raw serial frames and driver diagnostics

`DATA101` telegrams are recognised but deliberately ignored by the current decoder because the observed values did not provide a useful continuous torque curve.

---

# 2. System Requirements

The project uses:

- Ubuntu/Linux host or WSL2
- Docker
- ROS 2 Jazzy
- Python 3
- `pyserial`
- KL-TCG controller
- RS-232-to-USB adapter
- Controller serial port available as `/dev/ttyUSB0`

---

# 3. Start the Docker Container

From the host machine:

```bash
cd ~/fue_projekt
./start_docker_driver.sh
```

If the container already exists and is running:

```bash
docker exec -it ros2_bachelor bash
```

If the container exists but is stopped:

```bash
docker start ros2_bachelor
docker exec -it ros2_bachelor bash
```

Check the container status:

```bash
docker ps -a
```

---

# 4. Check the Serial Port

On the host machine:

```bash
ls -l /dev/ttyUSB*
```

Expected device:

```text
/dev/ttyUSB0
```

Check whether another process is using it:

```bash
sudo fuser -v /dev/ttyUSB0
```

Check USB detection:

```bash
dmesg | tail -n 30
```

Check the user groups:

```bash
groups
```

The user should normally belong to:

```text
dialout
```

If required:

```bash
sudo usermod -aG dialout $USER
```

Log out and log back in after modifying the group.

---

# 5. Check Python Syntax

Inside Docker:

```bash
python3 -m py_compile \
/home/robot/ros2_ws/src/screwdriver_driver/screwdriver_driver/kl_tcg_driver.py
```

No output means that the Python syntax is valid.

To check the GUI syntax as well:

```bash
python3 -m py_compile \
/home/robot/ros2_ws/src/screwdriver_driver/screwdriver_driver/screwdriver_gui.py
```

---

# 6. Build the ROS 2 Package

Inside Docker:

```bash
cd /home/robot/ros2_ws
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

Alternatively, using the project script:

```bash
cd /home/robot/scripts
./build.sh
```

Expected result:

```text
Starting >>> screwdriver_driver
Finished <<< screwdriver_driver

Summary: 1 package finished
```

---

# 7. Launch the Driver

## Option A – Using the launch script

Inside Docker:

```bash
cd /home/robot/scripts
./run_driver.sh
```

## Option B – Direct ROS 2 command

```bash
cd /home/robot/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run screwdriver_driver kl_tcg_driver
```

Expected message:

```text
Connected to KL-TCG on /dev/ttyUSB0 at 115200 baud
```

Keep this terminal open while using the driver.

---

# 8. Launch with a YAML Configuration

The node can load its ROS 2 parameters from:

```text
/home/robot/ros2_ws/src/screwdriver_driver/config/screwdriver_config.yaml
```

Launch it with:

```bash
cd /home/robot/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run screwdriver_driver kl_tcg_driver \
  --ros-args \
  --params-file \
  /home/robot/ros2_ws/src/screwdriver_driver/config/screwdriver_config.yaml
```

Verify the loaded parameters:

```bash
ros2 param get /kl_tcg_driver target_job
ros2 param get /kl_tcg_driver step_number
ros2 param get /kl_tcg_driver program_number
ros2 param get /kl_tcg_driver job_number
```

> The defaults directly declared in the current Python node use TS 20, TP 20 and Job 20.  
> A YAML file or the GUI can override them with Job 1, TP 1 and TS 1.

---

# 9. Launch the Monitoring Window

Open a second host terminal:

```bash
docker exec -it ros2_bachelor bash
```

Then:

```bash
cd /home/robot/scripts
./monitor.sh
```

The monitoring interface normally displays:

- controller state;
- last cycle result;
- controller configuration responses;
- driver events.

---

# 10. Launch the Configuration GUI

If `screwdriver_gui.py` is installed in `setup.py`, open another terminal inside Docker:

```bash
source /opt/ros/jazzy/setup.bash
source /home/robot/ros2_ws/install/setup.bash

ros2 run screwdriver_driver screwdriver_gui
```

The GUI allows the operator to:

- change speed;
- select CW or CCW direction;
- change the target number of turns;
- change the torque limit;
- change the delay;
- write TS, TP and Job;
- read TS parameters back;
- execute the configured Job;
- monitor controller state and final results.

---

# 11. Available ROS 2 Topics

Check the topics:

```bash
source /home/robot/ros2_ws/install/setup.bash
ros2 topic list
```

The driver publishes:

```text
/screwdriver/state
/screwdriver/result
/screwdriver/config
/screwdriver/raw_frame
/screwdriver/events
```

## `/screwdriver/state`

Source:

```text
REQ100 and DATA100
```

Contains:

- controller connection state;
- screwdriver connection state;
- enable status;
- controller mode;
- active Job;
- active Sequence;
- active Tightening Program;
- screw counter;
- stop code.

Display it with:

```bash
ros2 topic echo /screwdriver/state --field data
```

Example:

```text
STATE
-----
Controller : connected
Tool       : connected
Enabled    : yes
Mode       : STD
Job/Seq/TP : 01/01/01
Screws     : 00/01
Stop       : 0 (none)
```

## `/screwdriver/result`

Source:

```text
DATA100
```

Contains the final result of a completed cycle:

- OK, NG or OKALL status;
- final torque;
- torque unit;
- fastening time;
- rotations;
- cycle counter.

Display it with:

```bash
ros2 topic echo /screwdriver/result --field data
```

Example:

```text
LAST RESULT
-----------
Result     : OKALL
Torque     : 0.032 N.m
Time       : 1.092 s
Rotations  : 5.0
Cycle Nr.  : 63
```

A result is only republished when the `DATA100` cycle counter changes.

## `/screwdriver/config`

Contains:

- Job-selection acknowledgement;
- TS write acknowledgement;
- TP write acknowledgement;
- Job write acknowledgement;
- TS, TP or Job read-back.

Display it with:

```bash
ros2 topic echo /screwdriver/config --field data
```

## `/screwdriver/raw_frame`

Contains every complete serial telegram received by the node.

```bash
ros2 topic echo /screwdriver/raw_frame --field data
```

Use this topic to inspect:

```text
REQ100
DATA100
DATA101
ANS104
ANS121
ANS122
ANS123
ANS124
```

## `/screwdriver/events`

Contains:

- connection messages;
- transmitted commands;
- generic controller answers;
- decoding failures;
- unknown frames;
- serial communication errors.

```bash
ros2 topic echo /screwdriver/events --field data
```

---

# 12. Available ROS 2 Services

Check the services:

```bash
ros2 service list | grep screwdriver
```

Implemented services:

```text
/screwdriver/read_controller_config
/screwdriver/read_tool_status
/screwdriver/execute_job
/screwdriver/write_step
/screwdriver/write_program
/screwdriver/write_job
/screwdriver/read_setting
```

All services use:

```text
std_srvs/srv/Trigger
```

The service request itself contains no parameter fields. The required payload must first be stored in ROS 2 parameters.

---

# 13. Read the Controller Configuration

Call:

```bash
ros2 service call \
/screwdriver/read_controller_config \
std_srvs/srv/Trigger
```

The driver sends:

```text
CMD108
```

Monitor the controller answer:

```bash
ros2 topic echo /screwdriver/events --field data
```

Expected output:

```text
Sent command: {CMD108,...}
Controller answer: ANS108 | raw={ANS108,...}
```

---

# 14. Read the Tool Status

Call:

```bash
ros2 service call \
/screwdriver/read_tool_status \
std_srvs/srv/Trigger
```

The driver sends:

```text
CMD116
```

Monitor:

```bash
ros2 topic echo /screwdriver/events --field data
```

---

# 15. Select and Restart a Job

Set the Job and Sequence:

```bash
ros2 param set /kl_tcg_driver target_job 1
ros2 param set /kl_tcg_driver target_sequence 1
```

Then call:

```bash
ros2 service call \
/screwdriver/execute_job \
std_srvs/srv/Trigger
```

The driver sends:

```text
CMD104
```

Monitor the response:

```bash
ros2 topic echo /screwdriver/config --field data
```

Expected:

```text
JOB SELECTION RESULT
--------------------
Status : accepted by controller
```

> **Important**
>
> `CMD104` selects and restarts the requested Job from its beginning.
>
> When the external start connection CN1–CN2 is already closed, the screwdriver may begin rotating immediately.
>
> Treat `/screwdriver/execute_job` as a movement-triggering command whenever CN1–CN2 is active.

---

# 16. Write a Tightening Step

The corresponding command is:

```text
CMD121
```

Set the parameters:

```bash
ros2 param set /kl_tcg_driver step_number 1
ros2 param set /kl_tcg_driver step_name ROS001
ros2 param set /kl_tcg_driver step_speed_rpm 300
ros2 param set /kl_tcg_driver step_direction 0
ros2 param set /kl_tcg_driver step_turns 5.0
ros2 param set /kl_tcg_driver step_torque_nm 1.0
ros2 param set /kl_tcg_driver step_delay_s 0.8
```

Direction values:

```text
0 = CCW / reverse
1 = CW / forward
```

Write the step:

```bash
ros2 service call \
/screwdriver/write_step \
std_srvs/srv/Trigger
```

Monitor:

```bash
ros2 topic echo /screwdriver/config --field data
```

Expected:

```text
WRITE STEP RESULT
-----------------
Status : accepted by controller
```

---

# 17. Write a Tightening Program

The corresponding command is:

```text
CMD122
```

Example for TP 1 referencing TS 1:

```bash
ros2 param set /kl_tcg_driver program_number 1
ros2 param set /kl_tcg_driver program_name ROS001

ros2 param set /kl_tcg_driver program_step_1 1
ros2 param set /kl_tcg_driver program_step_2 0
ros2 param set /kl_tcg_driver program_step_3 0
ros2 param set /kl_tcg_driver program_step_4 0
ros2 param set /kl_tcg_driver program_step_5 0

ros2 param set /kl_tcg_driver program_okall_time_s 1.0
ros2 param set /kl_tcg_driver program_ok_time_s 1.0
ros2 param set /kl_tcg_driver program_okall_stop 0
ros2 param set /kl_tcg_driver program_ng_stop 0
```

Write TP 1:

```bash
ros2 service call \
/screwdriver/write_program \
std_srvs/srv/Trigger
```

Expected configuration response:

```text
WRITE PROGRAM RESULT
--------------------
Status : accepted by controller
```

---

# 18. Write a Job

The corresponding command is:

```text
CMD123
```

Example for Job 1, Sequence 1, TP 1, Repeat 1:

```bash
ros2 param set /kl_tcg_driver job_number 1
ros2 param set /kl_tcg_driver job_sequence_1_program 1
ros2 param set /kl_tcg_driver job_sequence_1_repeat 1

ros2 param set /kl_tcg_driver job_reverse_direction 0
ros2 param set /kl_tcg_driver job_reverse_force 5
ros2 param set /kl_tcg_driver job_reverse_speed_rpm 300
```

Write the Job:

```bash
ros2 service call \
/screwdriver/write_job \
std_srvs/srv/Trigger
```

Expected:

```text
WRITE JOB RESULT
----------------
Status : accepted by controller
```

The current driver writes one active Job Sequence. The remaining Job Sequence fields are written as zero.

---

# 19. Read Back a Tightening Step

Set:

```bash
ros2 param set /kl_tcg_driver read_select 1
ros2 param set /kl_tcg_driver read_number 1
```

Call:

```bash
ros2 service call \
/screwdriver/read_setting \
std_srvs/srv/Trigger
```

Monitor:

```bash
ros2 topic echo /screwdriver/config --field data
```

Example:

```text
TIGHTENING STEP READBACK
------------------------
Step number      : 1
Step name        : ROS001
Speed            : 300 rpm
Target condition : Thread / turns
Target turns     : 5.0
Target torque    : 1.000 N.m
Direction        : CCW / reverse
Delay            : 0.8 s
```

---

# 20. Read Back a Tightening Program

Set:

```bash
ros2 param set /kl_tcg_driver read_select 2
ros2 param set /kl_tcg_driver read_number 1
```

Call:

```bash
ros2 service call \
/screwdriver/read_setting \
std_srvs/srv/Trigger
```

Expected output:

```text
TIGHTENING PROGRAM READBACK
---------------------------
Program number : 1
Program name   : ROS001
Step 1         : 1
Step 2         : 0
Step 3         : 0
Step 4         : 0
Step 5         : 0
```

---

# 21. Read Back a Job

Set:

```bash
ros2 param set /kl_tcg_driver read_select 3
ros2 param set /kl_tcg_driver read_number 1
```

Call:

```bash
ros2 service call \
/screwdriver/read_setting \
std_srvs/srv/Trigger
```

Expected output:

```text
JOB READBACK
------------
Job number : 1
Total JS   : 01
Sequence   : 1
Program    : 1
Repeat     : 1
```

---

# 22. Complete Manual Configuration Workflow

Use this order:

```bash
# 1. Write TS
ros2 service call \
/screwdriver/write_step \
std_srvs/srv/Trigger

# 2. Write TP
ros2 service call \
/screwdriver/write_program \
std_srvs/srv/Trigger

# 3. Write Job
ros2 service call \
/screwdriver/write_job \
std_srvs/srv/Trigger

# 4. Read TS back
ros2 param set /kl_tcg_driver read_select 1
ros2 param set /kl_tcg_driver read_number 1

ros2 service call \
/screwdriver/read_setting \
std_srvs/srv/Trigger

# 5. Select and restart Job
ros2 service call \
/screwdriver/execute_job \
std_srvs/srv/Trigger
```

Before the last command, ensure that the screwdriver and work area are safe.

---

# 23. Parameter Validation

The current node validates these ranges before transmitting a command.

## Tightening Step

```text
step_number       1 ... 250
step_name         1 ... 6 characters
step_speed_rpm    160 ... 1000
step_direction    0 or 1
step_turns        0.1 ... 50.0
step_torque_nm    0.6 ... 3.0 N.m
step_delay_s      0.0 ... 9.9 s
```

## Tightening Program

```text
program_number          1 ... 99
program_step_1          1 ... 250
program_step_2 ... 5    0 ... 250
program_okall_time_s     0.0 ... 9.9
program_ok_time_s       0.0 ... 9.9
program_okall_stop      0 or 1
program_ng_stop         0 ... 9
```

## Job

```text
job_number                  1 ... 50
job_sequence_1_program      1 ... 99
job_sequence_1_repeat       1 ... 99
job_reverse_direction       0 or 1
job_reverse_force           0 ... 9
job_reverse_speed_rpm       160 ... 1000
```

---

# 24. Debugging Commands

## Check the running nodes

```bash
ros2 node list
```

Expected:

```text
/kl_tcg_driver
```

## Show all node parameters

```bash
ros2 param list /kl_tcg_driver
```

## Dump all current parameters

```bash
ros2 param dump /kl_tcg_driver
```

## Show all driver services

```bash
ros2 service list | grep screwdriver
```

## Check a topic type

```bash
ros2 topic type /screwdriver/result
```

## Check topic publishers and subscribers

```bash
ros2 topic info /screwdriver/result --verbose
```

## Show raw communication

```bash
ros2 topic echo /screwdriver/raw_frame --field data
```

## Show sent commands and errors

```bash
ros2 topic echo /screwdriver/events --field data
```

## Show controller answers

```bash
ros2 topic echo /screwdriver/config --field data
```

---

# 25. Troubleshooting

## Node not found

Error:

```text
Node not found
```

Check whether the node is running:

```bash
ros2 node list
```

Restart it if necessary:

```bash
cd /home/robot/scripts
./run_driver.sh
```

---

## Service unavailable

Check:

```bash
ros2 service list | grep screwdriver
```

Ensure the workspace was sourced:

```bash
source /opt/ros/jazzy/setup.bash
source /home/robot/ros2_ws/install/setup.bash
```

---

## Serial port not connected

Check the host:

```bash
ls -l /dev/ttyUSB0
sudo fuser -v /dev/ttyUSB0
```

Check whether the device was passed into Docker:

```bash
ls -l /dev/ttyUSB0
```

from inside the container.

---

## Container-name conflict

Error:

```text
Conflict. The container name "/ros2_bachelor" is already in use
```

Check:

```bash
docker ps -a
```

Start the existing container:

```bash
docker start ros2_bachelor
docker exec -it ros2_bachelor bash
```

Or remove it if it is no longer needed:

```bash
docker rm ros2_bachelor
```

---

## Old code is still running after modification

Rebuild and resource the workspace:

```bash
cd /home/robot/ros2_ws

colcon build --symlink-install

source install/setup.bash
```

Stop every old driver process before restarting:

```bash
ps aux | grep kl_tcg_driver
```

---

## Repeated DATA100 telegrams

The controller repeats `DATA100` until it receives `CMD100`.

The current node automatically sends `CMD100` after processing `DATA100`.

Check:

```bash
ros2 topic echo /screwdriver/events --field data
```

A serial or acknowledgement failure will appear there.

---

## DATA101 is visible but no live-torque topic exists

This is expected.

The current node contains:

```python
elif command == "DATA101":
    pass
```

Therefore, `DATA101` remains visible on:

```text
/screwdriver/raw_frame
```

but is not decoded or published as a separate live-torque topic.

---

## Service returned success but no controller confirmation appeared

The service response confirms that the driver validated and transmitted the command.

Controller acceptance arrives separately as:

```text
ANS104
ANS121
ANS122
ANS123
ANS124
```

Monitor:

```bash
ros2 topic echo /screwdriver/config --field data
ros2 topic echo /screwdriver/events --field data
```

---

# 26. Stop the System

Stop the driver:

```text
Ctrl + C
```

Stop the GUI:

```text
Ctrl + C
```

Leave Docker:

```bash
exit
```

Stop the container from the host:

```bash
docker stop ros2_bachelor
```

---

# Normal Startup Procedure

## Terminal 1 – Driver

```bash
cd ~/fue_projekt
./start_docker_driver.sh
```

If the container already exists:

```bash
docker start ros2_bachelor
docker exec -it ros2_bachelor bash
```

Then:

```bash
cd /home/robot/scripts
./build.sh
./run_driver.sh
```

## Terminal 2 – Monitor

```bash
docker exec -it ros2_bachelor bash

cd /home/robot/scripts
./monitor.sh
```

## Terminal 3 – GUI

```bash
docker exec -it ros2_bachelor bash

source /opt/ros/jazzy/setup.bash
source /home/robot/ros2_ws/install/setup.bash

ros2 run screwdriver_driver screwdriver_gui
```

The ROS 2 driver is then ready for monitoring, parameter configuration and Job execution.

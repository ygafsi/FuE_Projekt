syntax
python3 -m py_compile \
/home/robot/ros2_ws/src/screwdriver_driver/screwdriver_driver/kl_tcg_driver.py

trigger a program

ros2 service call /screwdriver/execute_job std_srvs/srv/Trigger

# FuE Project – KL-TCG ROS2 Driver

This document describes how to build, launch and test the current ROS2 driver for the KILEWS KL-TCG controller.

The current implementation supports:

- Serial communication with the KL-TCG controller (RS232)
- Controller state monitoring
- Final tightening result monitoring
- Reading controller configuration
- Reading tool status
- Selecting an existing Job and Sequence

---

# 1. Start the Docker Container

From the host machine:

```bash
cd ~/fue_projekt
./start_docker_driver.sh
```

If the container is already running:

```bash
docker exec -it ros2_bachelor bash
```

---

# 2. Build and Launch the Driver

## Terminal 1 (inside Docker)

```bash
cd /home/robot/scripts

./build.sh
./run_driver.sh
```

The driver should print:

```text
Connected to KL-TCG on /dev/ttyUSB0 at 115200 baud
```

---

# 3. Open the Monitoring Window

## Terminal 2

From the host:

```bash
docker exec -it ros2_bachelor bash
```

Inside Docker:

```bash
cd /home/robot/scripts

./monitor.sh
```

The monitoring window displays:

- Controller State
- Last Tightening Result

---

# 4. Available Topics

```bash
source /home/robot/ros2_ws/install/setup.bash

ros2 topic list
```

Main topics:

```text
/screwdriver/state
/screwdriver/result
/screwdriver/raw_frame
/screwdriver/events
```

---

# 5. Available Services

```bash
ros2 service list
```

Implemented services:

```text
/screwdriver/read_controller_config
/screwdriver/read_tool_status
/screwdriver/execute_job
```

---

# 6. Read Controller Configuration

```bash
ros2 service call \
/screwdriver/read_controller_config \
std_srvs/srv/Trigger
```

Monitor the communication:

```bash
ros2 topic echo /screwdriver/events --field data
```

Expected output:

```text
Sent command: {CMD108,...}
Controller answer: ANS108 ...
```

---

# 7. Read Tool Status

```bash
ros2 service call \
/screwdriver/read_tool_status \
std_srvs/srv/Trigger
```

---

# 8. Select a Job

The desired Job and Sequence are stored as ROS2 parameters.

Default values:

```text
Job      = 1
Sequence = 1
```

Change the parameters:

```bash
ros2 param set /kl_tcg_driver target_job 1
ros2 param set /kl_tcg_driver target_sequence 1
```

Then transmit the selection to the controller:

```bash
ros2 service call \
/screwdriver/execute_job \
std_srvs/srv/Trigger
```

Monitor the communication:

```bash
ros2 topic echo /screwdriver/events --field data
```

Expected output:

```text
Sent command: {CMD104,...}
Controller answer: ANS104 ...
```

> **Note**
>
> `CMD104` only selects an existing Job and Sequence.
>
> It does **not** start the screwdriver.

---

# 9. Check the Current Controller State

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

Screws     : 01/05

Stop       : 0 (none)
```

> Depending on the controller firmware, the displayed Job may update after the next tightening cycle.

---

# 10. Read the Last Tightening Result

```bash
ros2 topic echo /screwdriver/result --field data
```

Example:

```text
LAST RESULT
-----------
Result     : OK
Torque     : 0.302 N.m
Time       : 0.578 s
Rotations  : 5.0
Cycle Nr.  : 78
```

---

# 11. Debugging

## Show Raw Serial Frames

```bash
ros2 topic echo /screwdriver/raw_frame --field data
```

This topic displays every telegram received from the controller.

---

## Show Driver Events

```bash
ros2 topic echo /screwdriver/events --field data
```

This topic displays:

- transmitted commands
- controller responses
- serial communication errors

---

# 12. Stop the Driver

Stop the node:

```text
Ctrl + C
```

Leave the container:

```bash
exit
```

---

# Troubleshooting

## Check if another driver is running

```bash
ps aux | grep kl_tcg_driver
```

Only the `grep` process should appear.

---

## Check whether the serial port is free (host machine)

```bash
sudo fuser -v /dev/ttyUSB0
```

No process should be using the device.

---

# Current Driver Capabilities

The current implementation supports:

- RS232 communication with the KL-TCG controller
- Decoding of `REQ100` (controller state)
- Decoding of `DATA100` (final tightening result)
- Reading controller configuration (`CMD108`)
- Reading tool status (`CMD116`)
- Selecting an existing Job and Sequence (`CMD104`)

The current implementation does **not** yet support:

- Writing Tightening Steps (TS)
- Writing Tightening Programs (TP)
- Creating or modifying Jobs
- Starting the screwdriver by software (requires the hardware trigger CN1/CN2)
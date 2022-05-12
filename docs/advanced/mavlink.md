---
hide:
#  - navigation
---

# MAVLink
MAVLink is the underlying protocol used for communication with the drone as well as within the drone. This is the de facto industry standard used by many components and the software which run on them. As such, do note that alot of the common errors can be diagnosed via MAVLink messages and more advanced functionality such as high level autonomy are built upon it. With MAVLink, sensors and components can speak to each other through this messaging protocol, making integration of components on a drone much easier. Notably, this standard protocol also allows for packages such as Mavros to interface ROS with flight controllers to allow for autonomous flight related features.

*For missions accomplished within QGC(eg SAFMC D1 or purely GPS controlled missions in AAVC), knowledge in MAVlink is not really necessary. Nonetheless, having this knowledge will benefit your understanding and could help you leverage some of its useful features.*

A short list of notable hardware and software which support MAVLink
+ Any flight controller running
  + Ardupilot
  + PX4
+ Any computer running
  + QGroundControl
  + MissionPlanner
  + MAVROS
  + Program written with MAVSDK
+ Various Camera Gimbal/Gimbal Controllers supporting MAVLink
+ Arduino   

https://mavlink.io/en/about/implementations.html

## Degbugging PX4 using MAVLink Shell
The MAVLink Shell is the Nutt Shell (running on NuttX RTOS that PX4 is built upon) accessed through MAVLink.

[Heres a list of mavlink commands](https://dev.px4.io/v1.9.0/en/middleware/modules_command.html) to help with debugging PX4 with QGC (Ardupilot seems to have discontinued it). The listener <device> command is pretty useful to check if a sensor is publishing data(eg listener optical_flow, listener uavcan). Sensors can be manually started and stopped(eg tfmini start, tfmini status, tfmini stop).

## Mavlink Inspector
An important diagnostic tool easily overlooked is the mavlink inspector. This can be used to observe various mavlink values in real-times which is useful for tuning stuff like altitude hold/setpoint. Both GCS have the ability to graph multiple values live which are helpful to validate the changing values.

The full descriptions of [mavlink messages](https://mavlink.io/en/messages/common.html#MAV_CMD) can be found here. Understanding it is essential for autonomous flight modes with ROS or dronekit. Mavlink messages can be sent too in both [Mission Planner](https://ardupilot.org/dev/docs/commonmission-planner-command-line-interface-cli.html) and [QGC](https://docs.qgroundcontrol.com/en/analyze_view/mavlink_console.html) which can carry out any task such as arming or enabling range sensor drivers.

In Ardupilot, Mavlink inspector can be accessed in the temp menu by hitting ctrl-f. Alternatively, quite a lot of readings can be bound to the quick screen in Mission Planner(unlike QGC)
![logo](https://i.imgur.com/Gq7tatI.png)

In PX4, the mavlink inspector is found under the Logs Tab.
![logo](https://i.imgur.com/cUye0sY.png)

The Mavlink port can be broadcasted over a local network which allows you to connect remotely via another computer which is useful for debugging while viewing other sources. Ardupilot has this function built in via the mavlink option on the temp page but this can be carried out on px4 too.
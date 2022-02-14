---
hide:
  - navigation
---

# Introduction to ADVANCED Dronery

By this point, you should have built your workshop drone, flashed it with betaflight, and proceeded to maiden flight with it. However, to achieve autonomous or semi-autonomous functionality, a more powerful flight controller with a compatible firmware and more sensors are required for the drone to detect changes in its surroundings. This document will primarily focus on some of the required steps for autonomous flight and the interfacing of sensors with the two main autonomous flightstacks(Ardupilot and PX4).

**Disclaimer from the authors**  
This is a collection of notes taken down by past and present MRS members. This is by no means extensive of what is needed to be known for those competitions nor is it fully fact checked.

## Setting up Autonomous Flight Modes
Below are the required calibrations for autonomous flight. Importantly, tuning the parameters should not be done solely visually, but with the help of the live graphs and the logs. It is important for the flight controller to be able to respond correctly to reach its target/setpoint. When flying manually it might not be as apparent that parameters are incorrect as we have to benefit of correcting visually. Reading logs takes experience so do not be afraid to ask around for advice.

QGC Logs location            |  Mission Planner Logs Location
:-------------------------:|:-------------------------:
![logo](https://i.imgur.com/D4BYmMd.png) |![logo](https://i.imgur.com/xwjKWKZ.png)

Ardupilot logs can be reviewed on the app but for PX4 has this nice website [to review logs](https://logs.px4.io/).

Do note that in the case of the sudden power loss to the FCU, the logs will cutoff right after that point.

## MAVLink
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

### Degbugging PX4 using MAVLink Shell
The MAVLink Shell is the Nutt Shell (running on NuttX RTOS that PX4 is built upon) accessed through MAVLink.

[Heres a list of mavlink commands](https://dev.px4.io/v1.9.0/en/middleware/modules_command.html) to help with debugging PX4 with QGC (Ardupilot seems to have discontinued it). The listener <device> command is pretty useful to check if a sensor is publishing data(eg listener optical_flow, listener uavcan). Sensors can be manually started and stopped(eg tfmini start, tfmini status, tfmini stop).

### Mavlink Inspector
An important diagnostic tool easily overlooked is the mavlink inspector. This can be used to observe various mavlink values in real-times which is useful for tuning stuff like altitude hold/setpoint. Both GCS have the ability to graph multiple values live which are helpful to validate the changing values.

The full descriptions of [mavlink messages](https://mavlink.io/en/messages/common.html#MAV_CMD) can be found here. Understanding it is essential for autonomous flight modes with ROS or dronekit. Mavlink messages can be sent too in both [Mission Planner](https://ardupilot.org/dev/docs/commonmission-planner-command-line-interface-cli.html) and [QGC](https://docs.qgroundcontrol.com/en/analyze_view/mavlink_console.html) which can carry out any task such as arming or enabling range sensor drivers.

In Ardupilot, Mavlink inspector can be accessed in the temp menu by hitting ctrl-f. Alternatively, quite a lot of readings can be bound to the quick screen in Mission Planner(unlike QGC)
![logo](https://i.imgur.com/Gq7tatI.png)

In PX4, the mavlink inspector is found under the Logs Tab.
![logo](https://i.imgur.com/cUye0sY.png)

The Mavlink port can be broadcasted over a local network which allows you to connect remotely via another computer which is useful for debugging while viewing other sources. Ardupilot has this function built in via the mavlink option on the temp page but this can be carried out on px4 too.

## PID TUNING
Ensure that the PID tuning is stable by checking the response curves for the pitch and roll graphs before attempting to fly in mission/auto flight modes. Overshooting the setpoints will result in oscillations and crashes. It takes time and experience to get the tune right.
**However**, many other factors such as lack of dampening can cause excessive vibrations too, similarly loose screws or components that are not fastened down might generate issues. Visually inspect your drone and check for possible issues that might cause instability before attempting to get a perfect tune.

[PX4 PID Tuning Guide for Multicopters](https://docs.px4.io/v1.9.0/en/config_mc/pid_tuning_guide_multicopter.html)
There are 3 sections, do read through the first 2(even briefly) unless a crash is desired.
[PID Tuning Explained by Oscarliang](https://oscarliang.com/quadcopter-pid-explained-tuning/)
[Sample AAVC Test Flight with bad tuning and many oscillations](https://review.px4.io/plot_app?log=b51cbfc0-fb20-46d7-8d30-5252743e3eb2)

### The Ardupilot PID Autotune Shortcut
Ardupilot has this rather convenient feature that allows it to autotune the PID in an open area. Ensure that altitude mode works first before binding the autotune function to a switch. The drone will use step inputs to determine the ideal rates for a sharp response. The AUTOTUNE_AXES param can be changed if your drone does not have enough battery life to tune all axis at once. It is **highly recommended to use a GPS** as this will allow the drone to maintain position despite windy conditions when autotune is activated from position hold.

PID tuning only starts when both sticks are centered (ie the throttle will be at 50%), so ensure that altitude hold is tuned properly, more information below.

Here are some very brief notes regarding PID tuning, much more indepth information is available online.

### PID calibration procedure
Unlike racing drones it is impractical for larger, specialized drones to be tuned while flying FPV in acro so a method demanding less of the pilot will be briefly covered below. This process will be easier with one person flying and having a physical feel for how the drone response while the other reads the graphs and changes the params to suit the ideal response and pilots preference.

Firstly, use quick impulse inputs and tune for roll->pitch->yaw one axis at a time. These rapid inputs should only occur for less than a split second as we only want to see how fast it responds to the current input(ie inputting a impulse roll right input should cause the drone to momentarily tilt right. If it does not respond, the p is too low. If it starts to move right, then the roll input is too long and needs to be shorter.).

Repeat the impulse inputs a few times and observe the immediate response to the step input while discarding any external interferences(such as wind or lose parts causing movements). Tuning outdoors is unadvisable(aside from autotune) because of this. Do note that if tuning is conducted midair, always use the 5% increment button to avoid causing big changes in control to the drone and crashing.

For a balanced x-frame, you can expect the roll and pitch pids to be roughly the same, which speeds up tuning. For a H frame with the weight distributed along the y axis of the craft, the roll p-pid should be lower than the pitch p-pid as it is easier to roll. 

The pid graph in PX4 is under Tuning-> Advanced Perimeters. The estimated(Red) is the actual position of the craft while the setpoint(green) is what is desired, therefore the setpoint should hug the estimated as close as possible.

#### 1) P, maximum value possible
**_Response_**
Too high - high frequency oscillations(it will be both visible and audible)
Too low - vehicle reacts too slowly

*Increase value until right before it starts oscillating in high frequency* 

#### 2) D, minimum value possible
**_Dampening_**
Too high - motors become twitchy and hot(because it overcorrects too quickly, which creates high frequency noise)
Too low - overshoot after step input

*Find Minimum value required so that it corrects quickly(see from graph) even after large step inputs*

#### 3) I, balanced
**_Error_** 
Too high - slow oscillations ( as it slowly tries to correct)
Too low - drift, HIGH FREQUENCY OSCILLATION AFTER STEP INPUT

*Increase if drifts in wind/external conditions, decrease if it corrects too slowly/little*
*Recommended values are 0.3-0.5 on PX4 wiki*



Arguably, this is one of the hardest parts of setting up a drone as it requires experience to isolate and identify the actual response.

Isolate - ideally speaking, tuning should be done in an indoor environment without wind but since that is not always possible with bigger drones and lack of space, just be aware that external factors such as wind can cause the response to move on its own. Do not overcompensate for it.

Identify - One common issue is identifying high frequency oscillations from low frequency oscillations when first starting out. The default PIDs from PX4 are deliberately set low for safety but it can be more than doubled on some drones. The biggest tell of high frequency oscillations is that it should be very hard to even take off(unlike low frequency which is still often possible). The drone should both visibly and audibly be constantly vibrating and trying to correct, with little or not control as a result. 

As one might be unable to differentiate high from low frequency oscillations when tuning for the first time, the P tuning might be stopped at a much lower value than what the craft is actually capable of. Considering the size and price of the platform, it is understandable why this is more of an issue as compared to tuning an acro quad. Hence, it maybe useful to do a preliminary tuning of P,D,I and after it is sufficiently stable **continue trying to increase P** until it is uncontrollable.

The live graphs for PID tuning can be found from the advanced option under the tuning tab.
![logo](https://i.imgur.com/Vz8bNGC.png)
The parameters can be tuned on the fly incrementally.
![logo](https://i.imgur.com/XPvmYo9.png)

### Example tuning process

Here are some images of a tuning process that will perhaps help to judge the feedback of your drone. It is by no means ideal but should aid in helping you to read the graph if you are struggling with identifying what is going on. Do not be overly reliant on the graphs, visual identification and the pilot feedback are as important when tuning.

![logo](https://imgur.com/1Gx4XdM.png)
As previously mentioned, try inputting impulse commands with P and no I or D. In this picture, the response is smaller than the command. Hence, the P has to be increased for increased responsiveness of the drone and to ensure that the graphs are closer to one another. When the drone starts exhibiting high frequency oscillations, either reduce the P **or** start adding in D together with P if more responsiveness is still desired.

![logo](https://imgur.com/GkSbp29.png)
In this graph, the drone oscillates after an impulse input. More D is required to dampen the response quicker.

![logo](https://imgur.com/tvF2A5H.png)
This graph looks pretty okay but the response looks abit slow so the I can be reduced.

![logo](https://imgur.com/ZFJIylE.png)
TBC

## Compass
Work in progress

## Positioning and Navigation
It should self explanatory why positioning is important for autonomous flight modes. **DO ENSURE THAT ALL THE AUTONOMOUS FLIGHT MODE SPEEDS ARE TURNED DOWN** before attempting first flight.
### Outdoor
As compared to indoor navigation, autonomous outdoor navigation is relatively trivial as it can be accomplished with GPS position data and GPS waypoints so long as the drone has view of the sky throughout the mission to get GPS signals.
### Indoor
However, if the drone is required to fly in tunnels or buildings where GPS cannot attain lock, other methods to position and localize the drone are needed.

[This is a super handy link in ardupilot wiki](https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html) regarding commonly used sensors for indoor navigation. Amongst those which are used before are the [Intel T265](https://ardupilot.org/copter/docs/common-vio-tracking-camera.html) (which has a in-depth walkthrough in the link) and the [Optical Flow sensors](https://ardupilot.org/copter/docs/common-optical-flow-sensors-landingpage.html) (do ensure to check the optical flow setup link for the full instructions). Optical flow is particularly popular, with drones such as DJI utilizing it. It is preferred as external setup of beacons are not needed. Do note that because of the transforms conducted on the image, it is both sensor and lens dependent, so do be aware of it when using open source projects.

The Ardupilot wiki page mainly mentions optical or beacon based tracking solutions with the exception being cartographer with lidar(at time of writing)

Further details on specific optical flow configuration are given in the respective Ardupilot and PX4 sections.

## Altitude Hold
An important prerequisite to position hold is the altitude hold. Do note that (IIRC) both Ardupilot and PX4 do not have a param to change the radio controller throttle % at which it occurs and it always occurs when your radio controller is physically at 50% throttle. The settings altered simply changes the value of the throttle pwm value in which the flightstack tries to hover around. Hence, at 50% (+- whatever threshold is set) should allow it to hover and below 50% should activate a velocity based descent depending on how much the sticks are pushed down and above 50% should ascent the drone.

In Ardupilot, [Hover Throttle is automatically learnt](https://ardupilot.org/copter/docs/ac_throttlemid.html#ac-throttlemid) but can still be set manually if desired.

In PX4, always set your hover throttle **is not automatically learnt** so remember to lower than the actual thrust ratio to avoid banging the ceiling(especially if your craft is overpowered, ie less than 50% to hover).



With the desired performance of altitude hold explained, tuning for the z axis in altitude/position mode can occur.
First, attempt to hover the drone while observing the throttle value required in the Mavlink inspector. Set this value as the hover throttle value. (PX4: MPC_THR_HOVER). The params are found under MPC(multicopter position control) in PX4.

In PX4, hover throttle can be changed with this slider. There is a param for this value but more testing is needed to determine if the param changes anything else.
![logo](https://i.imgur.com/gHkHTEX.png)

Next, the hover throttle threshold can be set and the throttle gain can tuned to prevent oscillations up and down around hover. (PX4: MPC_Z)

**PLACEHOLDER IMAGE**

***Do note that PX4 has finnicky landing detection in attitude mode, so land in attitude mode, throttle down and switch to stabilized mode to disarm instead of switching modes midair and risking the drone shooting up.***

## Obstacle Avoidance

One important feature of autonomous flight modes is the ability to not crash into obstacles. While rudimentary implementations allow for collision detection, where it [stops infront of objects and multiple obstacle sensors for 360 coverage](https://ardupilot.org/dev/docs/code-overview-object-avoidance.html), increasingly [**ardupilot**](https://ardupilot.org/copter/docs/common-object-avoidance-landing-page.html) and [**px4**](https://docs.px4.io/master/en/computer_vision/collision_prevention.html) have implemented more parameters and algorithms to allow for dynamic path planning so that the drone can move around obstacles. These features are constantly being worked on, so do check the page for updates. In some cases in Ardupilot and [PX4](https://docs.px4.io/master/en/sensor/cm8jl65_ir_distance_sensor.html), specific sensors can be connected directly to the flight controller for obstacle avoidance but(currently) in most of the cases the obstacle avoidance is [done on a companion computer running ROS](https://ardupilot.org/copter/docs/common-realsense-depth-camera.html#common-realsense-depth-camera).

## Indoor Autonomous Flight for Ardupilot(WIP)
Ekf3 must be used instead of ekf2
Homepoint and ekf3 origin must be set on map before each flight, the drone icon should physically show up along with its heading.

Due to the lack of any GPS or markers, the sensor readouts must be rock solid. The [EKF algorithm is also based on the sensor readouts](https://ardupilot.org/copter/docs/common-apm-navigation-extended-kalman-filter-overview.html) so do ensure that there is no interference of the compass, baro, etc. Ensure that the compass does not drift when it is rapidly yawed, [do check this page for more information](https://ardupilot.org/copter/docs/common-magnetic-interference.html). Testing to be done with EMI shielding to make the sensors more stable.

On the topic of sensors and compasses, remember that you can change the orientation of the compass, the **primary compass used** and enable and disable certain compasses. However, [only one compass is used in flight and the other are used for error checking and redundancy](https://ardupilot.org/plane/docs/common-compass-setup-advanced.html#common-compass-setup-advanced).

(Unrelated) You can define which CAN Driver to use under a CAN port, [CAN_P1_DRIVER can be used on either physical port](https://ardupilot.org/copter/docs/parameters.html#can-p1-driver)

## Misc sensor reference for Ardupilot

### Misc reference for Ardupilot
#### ProfiCNC mini carrier board pinout
[Mini carrier board pinout](https://discuss.ardupilot.org/t/pixhawk-2-1-cube-mini-carrier-board-questions/33529/23)
[Alternate link](https://docs.cubepilot.org/user-guides/carrier-boards/mini-carrier-board)
*Do take note that the lack of certain ports on this board might result in incomptiablity of some sensors*

#### Misc Tips
BRD_SAFETYENABLE needs to be set to 0 for ESC calibration(else a safety switch needs to be connected),CBRK_IO on newer firmware

### Obstacle Avoidance Sensors
Most ultrasonic obstacle avoidance sensors comes with an analog value which is scaled based on distance and either a serial or I2C port depending on the version. I2C allows for multiple sensors to be chained together.

Looking at the RNGFNDx pin on the Ardupilot config, either the `ANALOG` can be used via the ADC port or `PWM` can be used when plugged into any of the auxiliary ports. However, due to the restriction of only one ADC port on the pixhawk, only one rangefinder can be used via ADC. The only sensor listed on the documentation capable of `PWM` output is the [Lidarlite](https://ardupilot.org/copter/docs/common-rangefinder-lidarlite.html). This leaves object avoidance with multiple sensors implementation via I2C or Serial. Using this method, since ADC or Analog is not used, the RNGFND param does not have to be changed. 

![logo](https://imgur.com/DcevgxM.png)

Most of the params are self explanatory and configuration can be found on the ardupilot wiki page itself. However, note that [certain libraries such as the Maxbiotix library might not be able to support multiple sensors](https://discuss.ardupilot.org/t/multiple-maxbotix-i2c-pixhawk-2-1-problem/35177/34), more testing is required.

[MaxBotix MB1200](https://www.maxbotix.com/Ultrasonic_Sensors/MB1200.htm) (I2C Version is MB1202)
The maxbiotix website only shows how to interface a single sensor for terrain following via [analog](https://www.maxbotix.com/articles/ultrasonic-sensors-pixhawk-ardupilot.htm).

The obstacle avoidance can be tested under the proximity button by hitting ctrl-f. [More information here](https://ardupilot.org/dev/docs/code-overview-object-avoidance.html).

### Optical Flow and Lidars

[**READ THIS FIRST**](https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html), in addition to the sensor specific setup.

Do note that similar to PX4. A downward facing distance sensor is **required** for optical flow.

Optical flow sensors make use of a downward facing camera to attempt to help the drone position hold while Lidars emits a laser which reflects in order to give a drone altitude data. Sensors like the HereFlow(optical flow) has [relatively straight forward config](https://ardupilot.org/copter/docs/common-hereflow.html) and even contains a lidar on board for altitude hold. Do note that the config on there HereFlow page is greatly lacking, refer to the [PX4 Flow](https://ardupilot.org/copter/docs/common-px4flow-overview.html#upgrade-the-px4flow-sensor-s-firmware) page for the full configuration. **Additionally there is an optical calibration button by hitting ctrl-f**. Additionally, the params for the HereFlow are pinned to the top of the [cubepilot forums](https://discuss.cubepilot.org/t/hereflow-setup-instructions-alpha-batch/341) along with a pdf containing the orientation of the board and under important details.

Also, the RNG_FND1 related parameters corresponds to CAN port 1 so ensure that the parameters for it are configured correctly if a different port is used.

For longer ranges, other lidars such as the terranger can be used to help the drone maintain altitude. Importantly, **EKF2_ALT_SOURCE**  has to be changed in order for the flow and lidar to be utilized in ekf caculations.

![logo](https://imgur.com/9Nb6Ywn.png)

This sensors are important for doing autonomous indoor flight as an unstable drone platform with no position hold will crash easily. Also, ensure that the PID tuning works well in position hold as the drone will oscillate and crash if the PID tune is bad. A safe way to avoid this is by tuning manually first and running PID Autotune to determine what the flight controller deems to be the ideal PIDs.

For the flow to be used a position hold, the flight mode [flowhold](https://ardupilot.org/copter/docs/flowhold-mode.html) must be used.

![logo](https://imgur.com/o5U2SfR.png)
Here are the list of the flow params.

![logo](https://imgur.com/2CvDwOD.png)
*Example of bad tracking*

## PX4 Reference

### Full list of Config

<http://docs.px4.io/v1.9.0/en/advanced_config/parameter_reference.html>

### Flight Modes

+ Manual/Stabilized - same for multi rotors, transmit pitch and roll commands to output mixer, output mixer also returns drone to an angle of zero when joysticks are centered.
+ Acro - transmit angular rates(how fast to go to an angle)
+ Position - Same as manual/stabilized but tries to correct for wind. May cause crash on larger drones if PID is not tuned properly as will start oscillating. **Exiting Mission mode through max throttle will automatically [change the mode to position mode](https://github.com/PX4/px4_user_guide/blob/master/en/getting_started/flight_modes.md)**

Reference: <https://dev.px4.io/v1.9.0/en/concept/flight_modes.html>

### GPS and EKF
The Estimation and Control Library (ECL) uses an Extended Kalman Filter (EKF) algorithm to process sensor measurements and provide an estimate of the following states:

-	Quaternion defining the rotation from North, East, Down local earth frame to X,Y,Z body frame
-	Velocity at the IMU - North,East,Down (m/s)
-	Position at the IMU - North,East,Down (m)
-	IMU delta angle bias estimates - X,Y,Z (rad)
-	IMU delta velocity bias estimates - X,Y,Z(m/s)
-	Earth Magnetic field components - North,East,Down (gauss)
-	Vehicle body frame magnetic field bias - X,Y,Z (gauss)
-	Wind velocity - North,East (m/s)

Basically, the filters that process the sensor data.
EKF2 processes and corrects for errors generated by aerodynamic disturbances caused by vehicle wind relative velocity and orientation.
ECL library can be used with better performance but requires knowledge in tuning kalman filters.

[EKF Tuning](https://docs.px4.io/v1.9.0/en/advanced_config/tuning_the_ecl_ekf.html)

### GPS EKF Params
| Metric               | Minimum required                                             | Average Value | Units | Notes                                                        |
| -------------------- | ------------------------------------------------------------ | ------------- | ----- | ------------------------------------------------------------ |
| eph                  | 3 ([EKF2_REQ_EPH](https://docs.px4.io/v1.9.0/en/advanced_config/parameter_reference.html#EKF2_REQ_EPH)) | 0.8           | m     | Standard deviation of horizontal position error              |
| epv                  | 5 ([EKF2_REQ_EPV](https://docs.px4.io/v1.9.0/en/advanced_config/parameter_reference.html#EKF2_REQ_EPV)) | 1.5           | m     | Standard deviation of vertical position error                |
| Number of satellites | 6 ([EKF2_REQ_NSATS](https://docs.px4.io/v1.9.0/en/advanced_config/parameter_reference.html#EKF2_REQ_NSATS)) | 14            | -     |                                                              |
| Speed variance       | 0.5                                                          | 0.3           | m/s   |                                                              |
| Fix type             | 3                                                            | 4             | -     |                                                              |
| hpos_drift_rate      | 0.1 ([EKF2_REQ_HDRIFT](https://docs.px4.io/v1.9.0/en/advanced_config/parameter_reference.html#EKF2_REQ_HDRIFT)) | 0.01          | m/s   | Drift rate calculated from reported GPS position (when stationary). |
| vpos_drift_rate      | 0.2 ([EKF2_REQ_VDRIFT](https://docs.px4.io/v1.9.0/en/advanced_config/parameter_reference.html#EKF2_REQ_VDRIFT)) | 0.02          | m/s   | Drift rate calculated from reported GPS altitude (when stationary). |
| hspd                 | 0.1 ([EKF2_REQ_SACC](https://docs.px4.io/v1.9.0/en/advanced_config/parameter_reference.html#EKF2_REQ_SACC)) | 0.01          | m/s   | Filtered magnitude of reported GPS horizontal velocity.      |

## Arming Checks
*EKF Preflight Checks/Errors
The following errors (with associated checks and parameters) are reported by the EKF (and propagate to QGroundControl):*

**PREFLIGHT FAIL: EKF HGT ERROR:**

-	This error is produced when the IMU and height measurement data are inconsistent.
-	Perform an accel and gyro calibration and restart the vehicle. If the error persists, check the height sensor data for problems.
-	The check is controlled by the COM_ARM_EKF_HGT parameter.

**PREFLIGHT FAIL: EKF VEL ERROR:**

-	This error is produced when the IMU and GPS velocity measurement data are inconsistent.
-	Check the GPS velocity data for un-realistic data jumps. If GPS quality looks OK, perform an accel and gyro calibration and restart the vehicle.
-	The check is controlled by the COM_ARM_EKF_VEL parameter.

**PREFLIGHT FAIL: EKF HORIZ POS ERROR:**

-	This error is produced when the IMU and position measurement data (either GPS or external vision) are inconsistent.
-	Check the position sensor data for un-realistic data jumps. If data quality looks OK, perform an accel and gyro calibration and restart the vehicle.
-	The check is controlled by the COM_ARM_EKF_POS parameter.
	

**PREFLIGHT FAIL: EKF YAW ERROR:**

-	This error is produced when the yaw angle estimated using gyro data and the yaw angle from the magnetometer or external vision system are inconsistent.
-	Check the IMU data for large yaw rate offsets and check the magnetometer alignment and calibration.
-	The check is controlled by the COM_ARM_EKF_POS parameter

**PREFLIGHT FAIL: EKF HIGH IMU ACCEL BIAS:**

-	This error is produced when the IMU accelerometer bias estimated by the EKF is excessive.
-	The check is controlled by the COM_ARM_EKF_AB parameter.

**PREFLIGHT FAIL: EKF HIGH IMU GYRO BIAS:**

-	This error is produced when the IMU gyro bias estimated by the EKF is excessive.
-	The check is controlled by the COM_ARM_EKF_GB parameter.

**PREFLIGHT FAIL: ACCEL SENSORS INCONSISTENT - CHECK CALIBRATION:**

-	This error message is produced when the acceleration measurements from different IMU units are inconsistent.
-	This check only applies to boards with more than one IMU.
-	The check is controlled by the COM_ARM_IMU_ACC parameter.

**PREFLIGHT FAIL: GYRO SENSORS INCONSISTENT - CHECK CALIBRATION:**

-	This error message is produced when the angular rate measurements from different IMU units are inconsistent.
-	This check only applies to boards with more than one IMU.
-	The check is controlled by the COM_ARM_IMU_GYR parameter.

**PREFLIGHT FAIL: COMPASS SENSORS INCONSISTENT - CHECK CALIBRATION:**

-	This error message is produced when the difference in measurements from different compass sensors is too great.
-	It indicates bad calibration, orientation or magnetic interference.
-	This check only applies to when more than one compass/magnetometer is connected.
-	The check is controlled by the COM_ARM_MAG parameter.

**PREFLIGHT FAIL: EKF INTERNAL CHECKS:**

-	This error message is generated if the innovation magnitudes of either the horizontal GPS velocity, magnetic yaw, vertical GPS velocity or vertical position sensor (Baro by default but could be range finder or GPS if non-standard parameters are being used) are excessive. Innovations are the difference between the value predicted by the inertial navigation calculation and measured by the sensor.
-	Users should check the innovation levels in the log file to determine the cause. These can be found under the ekf2_innovations message. Common problems/solutions include:
    - IMU drift on warmup. May be resolved by restarting the autopilot. May require an IMU accel and gyro calibration.
    - Adjacent magnetic interference combined with vehicle movement. Resolve my moving vehicle and waiting or re-powering.
    - Bad magnetometer calibration combined with vehicle movement. Resolve by recalibrating.
    - Initial shock or rapid movement on startup that caused a bad inertial nav solution. Resolve by restarting the vehicle and minimizing movement for the first 5 seconds.


### Other Parameters
The following parameters also affect preflight checks.

**COM_ARM_WO_GPS**  
The `COM_ARM_WO_GPS` parameter controls whether or not arming is allowed without a global position estimate.

- 1 (default): Arming is allowed without a position estimate for flight modes that do not require position information (only).
- 0: Arming is allowed only if EKF is providing a global position estimate and EFK GPS quality checks are passing

**COM_ARM_EKF_YAW**  
The COM_ARM_EKF_YAW parameter determines the maximum difference (in radians) between the navigation yaw angle and magnetic yaw angle (magnetometer or external vision) allowed before preflight checks fail. The default value of 0.5 allows the differences to be no more than 50% of the maximum tolerated by the EKF and provides some margin for error increase when flight commences. It can fail if the yaw gyro has a large offset or if the vehicle is moved or rotated in the presence of a bad magnetic interference or magnetometer calibration.

## PX4 Sensors

Certain important advice.

**Ensure that the EKF2_AID_MASKS is changed to allow for visual or gps fusion**

The 1.10 firmware allow for configuration on sensors on multiple port instead of only running on the default one. For example, the [SENS_TFMINI_CFG](https://docs.px4.io/v1.9.0/en/advanced_config/parameter_reference.html#SENS_TFMINI_CFG) parameter. However, do note that this selection is based on the **SERIAL PORTS ON THE ORIGINAL PIXHAWK 1** (https://www.mathworks.com/help/supportpkg/px4/ref/port-mapping-for-serial.html). So, if you were using the GPS2 port on the pixhawk 2, which is the /dev/ttyS6 port, you would need to choose the relevant S6 port on the pixhawk 1 which is serial 4. (The UART port on the pixhawk 4 is serial 4 too).

Take note that you can still manually start the sensor with <sensorname start -d /dev/ttyS6> but if the correct serial port is chosen it should automatically start everytime.

### [TF Mini](https://docs.px4.io/v1.9.0/en/sensor/tfmini.html)

By default on 1.8.2, PX4 reads the rangefinder port at **SERIAL4/GPS2** which corresponds to the/dev/tty/S6. the port can be rebound with SENS_TFMINI_CFG on later versions.

Ensure that SENS_EN_TFMINI is activated depending on firmware version. RNG AID should be enabled too.
Use the mavlink console to probe if the driver is running.

``` shell
tfmini start -d /dev/tty/S6
tfmini status
```

Related params

``` shell
EKF2_HGT_MODE  - changes the primary height data source, still uses other for fusing
EKF2_RNG_AID -can be enabled to reduce the barometer inaccuracies caused by ground effect if the range sensor is not the primary sensor.
MPC_ALT_MODE - follow altitude or terrain
```

Use the listener distance_sensor command to test if it still works. A distance_sensor tab should pop up too.

[Heres a useful link for pixhawk fc serial port mapping](https://www.mathworks.com/help/supportpkg/px4/ref/port-mapping-for-serial.html)

### Optical Flow
Before attempting to implement optical flow for the first time, read through [this entire page](https://docs.px4.io/v1.9.0/en/sensor/optical_flow.html). Aside from the obvious stuff like ensuring that the orientation and the offsets of the optical flow sensor is correct, **ensure that a separate downward facing rangefinder is used**(barometer is not allowed) and that integrated_xgyro and integrated_x,.. are the same. For sensors without an integrated imu, integrated_xgyro will show nan and integrated_x will not correspond with HIGHRES_IMU/xgyro.

**If connected to PX4 via (USB) MAVLink the Optical Flow device must publish to the OPTICAL_FLOW_RAD topic, and the distance sensor must publish to the DISANCE_SENSOR topic.** Only then can fusing occur. To view this topic over telem, the param MAV_1_MODE can be changed from normal to onboard to force all the messages to be sent. This can be checked with the command *mavlink status*. Do note that this causes other issues such as inability to pick up impulse inputs on the graph during PID Tuning, so any mavlink driver is set to this during tuning.

Aside from that, diagnosing flow issues are abit harder due to the lack of feedback in PX4. Do check your 1)orientation 2)altitude(needs to be high enough) and 3)mpc_xy_vel pids 4) flow sensor offset with [EKF2_OF_POS](https://docs.px4.io/master/en/advanced_config/parameter_reference.html#EKF2_OF_POS_X). If tuning is still required, try reducing the mpc_xy_vel_p to 0.08. If that does not help, try changing mpc_xy_vel_d before touching the other mpc params but do note that it should not need to be changed much as the defaults should work okay with the .

#### Hereflow
Take note that on pre 2019 pixhawk 2.1 boards the can 1 and can 2 boards are swapped. The hereflow should be plugged in on the can 1 port by default.

The following links contain the required perimeters for it.

+ [Hereflow](https://discuss.px4.io/t/hereflow-sensor-with-uavcan-not-responding/14694)
+ [PMW3901](https://github.com/PX4/px4_user_guide/blob/c03a25be2724e5d89e9ece47425423d5cd100a1d/en/sensor/pmw3901.md)

Do note that as of 1.10 the downward rangefinder does not seem to be supported. Hence, a seperate downward facing rangefinder might be needed.

The status of the hereflow can be checked with the mavlink commands `UAVCAN status` and `listener optical_flow`. Do note that the optical flow mavlink messages are not broadcasted over telem(in PX4) by default to save bandwidth.

 [Do note that the x and y axis in the hereflow manual seems to be swapped in px4](https://discuss.cubepilot.org/t/hereflow-setup-instructions-alpha-batch/341) 

Personally, I had issues getting the hereflow to work in the horizontal position so I placed it in the vertical position with the circle on the [pmw3901](https://docs.px4.io/master/en/sensor/pmw3901.html) facing the back.



### Here+/2 GPS
**Considerations**
*These errors show up as generic errors in qgc and is not diagnosable via logs*

- Ideally the base and rover antenna to have clear view of the sky that is 30 degrees above horizon.
- To attain GPS lock look for open spaces with view of sky instead of indoor areas
- **Place GPS away from other electronic devices which generate noise. Eg, FC, other antennas, wifi antennas, power transformers  and anything that emits EM/radio waves.** <https://ardupilot.org/copter/docs/common-magnetic-interference.html#common-magnetic-interference>
- Likewise with the Pixhawk FCU, dampening might be required

**Time-To-First-Fix**

|123 | GPS & GLONASS | GPS & BeiDou | GPS|
|-------------|-----|-----|-----|
|Cold Start|26S|28S|29S|
|Hot Start|1S|1S|1S|
|Aided Starts|2S|3S|2S|

#### Here+ /Here+ v2 LED Meaning
**[PROOFREADING NEEDED]There are 3 LEDs on the Here2, two at the side which are the UI LED and one in the center which should flash red with an emergency stop button in the center. If the two UI LEDs are not flashing, chances are [the mode switch](http://www.hex.aero/wp-content/uploads/2019/03/Here2-CAN-Instruction.pdf) within the here2 (casing) is in I2C Mode(default) instead of CAN Mode and swap to I2C cables. This I2C and CAN modes are for the compass only, GPS is still connected via Serial**.  

On PX4 this is information is vital as the errors are not very clear or displayed. However Ardupilot does display more GPS errors which makes it easier to diagnose even without LEDs.

- Flashing red and blue: Initializing sensors. Place the vehicle still and level while it initializes the sensors.
- Flashing blue: Disarmed, no GPS lock. Auto-mission, loiter and return-to-launch flight modes require GPS lock
- Solid blue: Armed with no GPS lock
- Flashing green: Disarmed (ready to arm), GPS lock acquired. Quick double tone when disarming from the armed state.
- Fast Flashing green: Same as above but GPS is using SBAS (so should have better position estimate)
- Solid green: with single long tone at time of arming: Armed, GPS lock acquired. Ready to fly!
- Double flashing yellow: Failing pre-arm checks (system refuses to arm)
- Single Flashing yellow: Radio failsafe activated
- Flashing yellow - with quick beeping tone: Battery failsafe activated
- Flashing yellow and blue- with high-high-high-low tone sequence (dah-dah-dah-doh): GPS glitch or GPS failsafe activated
- Flashing red and yellow: EKF or Inertial Nav failure
- Flashing purple and yellow: Barometer glitch Solid Red: Error，usually due to the SD card（re-plug or place the SD card to solve）,MTD or IMU，you may check the SD card and have a look at BOOT.txt for boot message analysis
- Solid red with SOS tone sequence: SD card missing or SD card bad format
- No light when power on: No firmware，firmware lost，SD card missing or bad format（ac3.4 or higher version）

<https://docs.cubepilot.org/user-guides/here+/here+v2-user-manual>

#### LED Status

#### UI LED - On I2C device (GPS LED, same as here2 LED)
#### Indicates Readiness of Flight
- **[Solid Blue] Armed, No GPS Lock:** Indicates vehicle has been armed and has no position lock from a GPS unit. When vehicle is armed, PX4 will unlock control of the motors, allowing you to fly your drone. As always, exercise caution when arming, as large propellers can be dangerous at high revolutions. Vehicle cannot perform guided missions in this mode.
- **[Pulsing Blue] Disarmed, No GPS Lock:** Similar to above, but your vehicle is disarmed. This means you will not be able to control motors, but all other subsystems are working.
- **[Solid Green] Armed, GPS Lock:** Indicates vehicle has been armed and has a valid position lock from a GPS unit. When vehicle is armed, PX4 will unlock control of the motors, allowing you to fly your drone. As always, exercise caution when arming, as large propellers can be dangerous at high revolutions. In this mode, vehicle can perform guided missions.
- **[Pulsing Green] Disarmed, GPS Lock:** Similar to above, but your vehicle is disarmed. This means you will not be able to control motors, but all other subsystems including GPS position lock are working.
- **[Solid Purple] Failsafe Mode:** This mode will activate whenever vehicle encounters an issue during flight, such as losing manual control, a critically low battery, or an internal error. During failsafe mode, vehicle will attempt to return to its takeoff location, or may simply descend where it currently is.
- **[Solid Amber] Low Battery Warning:** Indicates your vehicle's battery is running dangerously low. After a certain point, vehicle will go into failsafe mode. However, this mode should signal caution that it's time to end this flight.
- **[Blinking Red] Error / Setup Required:** Indicates that your autopilot needs to be configured or calibrated before flying. Attach your autopilot to a Ground Control Station to verify what the problem is. If you have completed the setup process and autopilot still appears as red and flashing, there may be another error.


#### Status LED (on FC)

| Color | Label | Bootloader usage | APP usage |
| ----- | ----- | ---------------- | --------- |
|Blue	|ACT (Activity)	|Flutters when the bootloader is receiving data	|Indication of ARM state|
|Red/Amber	|B/E (In Bootloader / Error)	|Flutters when in the bootloader	|Indication of an ERROR|
|Green	|PWR (Power)	|Not used by bootloader	|Indication of ARM state|

<https://docs.px4.io/v1.9.0/en/getting_started/led_meanings.html>

#### Other Params
 UAVCAN_ENABLE - For GPS via CAN (PX4)

## Misc Tips

In PX4 v1.8.2, calibrating esc does not always change the motor pwm_min or set one that is low enough. Do change it manually if the drone still spins too fast when arming.

In particular, ESCs like the ones on the DJI Snail seems to have issues calibrating with ardupilot and px4. If the other methods in the snail manual do not work, try calibrating each esc manually with a PWM/Servo tester

For reference, at min pwm, motors should not stop spinning when drone is tilted by 60 deg to each side. <https://docs.px4.io/v1.9.0/en/config_mc/pid_tuning_guide_multicopter.html>

On px4 reflashing to different software versions does not necessarily reset all parameters, ensure reset to factory default is also used

On pixhawk 2.1, battery voltage/current is only read from the first power port, second port will power the device without any readings.

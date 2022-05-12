---
hide:
#  - navigation
---

# Autonomous and Semi-Autonomous Flight

## Setting up Autonomous Flight Modes
Below are the required calibrations for autonomous flight. Importantly, tuning the parameters should not be done solely visually, but with the help of the live graphs and the logs. It is important for the flight controller to be able to respond correctly to reach its target/setpoint. When flying manually it might not be as apparent that parameters are incorrect as we have to benefit of correcting visually. Reading logs takes experience so do not be afraid to ask around for advice.

QGC Logs location            |  Mission Planner Logs Location
:-------------------------:|:-------------------------:
![logo](https://i.imgur.com/D4BYmMd.png) |![logo](https://i.imgur.com/xwjKWKZ.png)

Ardupilot logs can be reviewed on the app but for PX4 has this nice website [to review logs](https://logs.px4.io/).

Do note that in the case of the sudden power loss to the FCU, the logs will cutoff right after that point.

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
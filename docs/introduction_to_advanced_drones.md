# Disclaimer
This is a collection of notes I took down. Some of the information might be relevant to regular MRS drone competitions, SAFMC and AAVC. This is by no means extensive of what is needed to be known for those competitions or fully fact checked.

# AAVC Reference

[Facebook for announcements](https://www.facebook.com/groups/1014035222041487/)
[Website for rules](https://iaai.asia/aavc_Rule.html)

**Admin**

- Update prof about design and purchasing of items
- Inform Crystal about overseas trip (once the team lineup is complete), there is alot of admin forms that need to be settled so do this before hand
- To wrap the drone, get bubble wrap from ARMS office

**Misc Rules and tips**

- No autonomous payload(guided or mothership)
- Rules might change on the spot
- Clarify the presentation venue and length, it was 3 minutes for AAVC 2020
- Spline waypoints are cool but you probably wont use it
- Mission Planner will randomly delete the takeoff or first command. If failure to change to auto mode error comes up check this. Use simulation tab to verify flight plan.
- 3 Levels of Arbitrarily defined Autonomy for AAVC 2020,0 points for full manual, 2 points for waypoint(lvl 1), 4 for dynamically generated waypoints(lvl 2), 6 for automatically generated waypoints(lvl 3) which refers to pushing a single button or something like that.
- **Do not sacrifice points for autonomy over accuracy (if the former has less total points). If your CV solution cannot detect it reliably just take over and drop it near the target**
- Digital and Analog video transmission systems can interfere with each other, test before using
- It might be required to perform EMI shielding on certain electronics such as the Jetson or RPI to prevent it affecting the GPS, if that still does not work, increase the height of the GPS(away from the frame) with standoffs
- There will be multiple mannequins, be sure to check with the organizer the exact search area
- A cloth will be pasted on the injured area and the correct one will be face up
- With analog video(and possible digital) the mannequins cannot be identified at 20m. The drone has to descent either manually or autonomously for human or AI detection as it is simply too small(with the default wide angle cameras). For AAVC 2020 the altitude was 2-45m not 20-45m.

## Specing
For AAVC 2020 we speced the drone using the weight of the payload and the rough weight of the drone with generic parts. Then, we searched for efficient motors and high density batteries to fit the drone. The propellers and ESC are based on the motor requirements and the previously tested controllers were used for the FC and GPS. However, this might not be the ideal way to do it.

Ecalc Guide: https://ecalc.ch/calcinclude/help/xcoptercalctutorial.htm

## Some personal thoughts on frame and drone type in 2019
IMO with motors becoming more powerful and efficient such as the T-Motor U line, at least for a drone with an overall weight of 15kg it makes more sense to use four of these motors in a quad configuration(instead of a hexa or octaquad) to reduce the power draw, complexity and weight. To achieve this efficiency, large propellers have to be used which increases the frame size. This makes CF Rods/Square Tubes an ideal choice of material(due to its accessibility and ease of fabrication) as CF plates are not stiff enough and heavy in larger sheets which makes it impractical to use as a frame the larger it gets. The use of CF Rods in commercial drone with large propellers such as the DJI Matrice and the Freefly Alta X shows that perhaps using CF Rods as arms might be the effective option.

Two of the more common motor designs for the quadrotor class are the x-frame and the h-frame designs. These configurations do not directly define the body of the drones but influence the design of the supporting structures and hence indirectly the weight.
Though there are many variations of motor configurations involving different distances between the motors, IMO the difference is negligible when flying missions as the weight distribution matters more. 

![logo](
https://i.imgur.com/UGWarda.jpg)

IMO the x-frame configuration and its variants are harder to use as an test/mission platform over a h-frame/box-frame due to the following reasons.
1) The h-frame/box-frame is more rigid as the body has more connected rods/joints than a x-frame. Trying to remedy this on a x-frame with more rods turns the design into effectively a h-frame. This is important considering the drone has to withstand abuse and crashes. This also ties in to the relatively easier construction/CAD of the h-frame which does not need to have angled parts or a fixed base plate unlike the x frame.
2) The h-frame offers a large mounting area in between the two CF body rods while mounting on a x-frame arguably requires more planning as the base plate will have to be recut (or remoulded) to mount components differently.

Dont take this as META, if you are reading this you probably are just starting out with drones so its okay to try something more tested but experimental drones will teach you more at the end of the day so dont be afraid to expierement.

## Checks
### Assembly Checks
- Visual Inspection
- Check Motors and ESC, number/pin and direction; Calibration
- Calibrate all sensors(accelerometer, level, magnetometer/compass)

### Preflight Checks(**Do in reverse after Landing**)
- Visual Inspection for missing or cracked/broken parts
- Mechanical parts secured, arms not wobbling
- Check that motor screws are of correct length and not overtightened, can cause scraping feeling if screws are too long and scrapes the motor housing
- Check FPV Camera housing taken out
- Check Mode Switch
- Check Payload Switch
- Mount propellers and bushing(if applicable)
- Check for any errors in Mission Control/QGC
- Check FPV Camera Feed
- Check clear of people and nearby hazards

## Recommended Roles

Pilot - flies drone/ operates RC transmitter
Backup Pilot - might be required standing beside to the pilot for long range mission where the pilot flies FPV and the backup pilot can take over during LOS
Ground Control - Observes and provides audio feedback to pilot regarding flight status, flight mode,  battery, error states, etc
Spotter - ensures safety of other people and the surrounding, eg weather condition, birds, other drones, intervenes only if there is threat to mission or nearby people
FPV - for spotting objects/mannequin during mission

# Mission Planner vs QGC

There are differences between PX4 and Ardupilot and their target platforms which can be found online. To simplify things for now all you need to know is that Ardupilot is a flight control software in which Mission Planner is the program you download to configure it while PX4 is yet another flight control software in which QGroundControl(QGC) is the program used to configure it. Hardware wise usually the same flight controllers are compatible with both of them but Ardupilot seems to support newer sensors faster.

In AAVC 2020, we used Ardupilot for two main reasons

- 1) Auto PID Tuning on Ardupilot
  Ardupilot allows for automatic PID Tuning which can be bound to a switch under the config/tuning-> extended tuning page. Param AUTOTUNE_AXES can be set to individually tune one axis at a time so that the battery does not run flat. The drone has to be landed while in PID Mode for the PID to be saved.[Link](https://ardupilot.org/copter/docs/autotune.html)
  This saves alot of time normally spent tuning(especially as beginners) as it is hard to tune large drones in SUTD

- 2) Servo Release without Mixing
  Ardupilot has many presets for mixing the main and auxiliary outputs as unlike PX4 which required mixing for actuation of a servo. By making use of the camera shutter functionality, the payload can be triggered via DIGICAM_CONTROL during missions or bound to a switch. On the pixhawk cube (2), RC8 is Main 8 and RC9 is Aux 1, RC10 is Aux 2, etc... [Link](https://ardupilot.org/copter/docs/common-servo.html). Importantly, the RC ports on the pixhawk can all be set as either input or output. The RCx params control the input behaviour, but when RCx_OPTION is set to 0, the port is set as an an output with the output behaviours being controlled by [the servo params](https://ardupilot.org/copter/docs/common-rcoutput-mapping.html). This includes the 4 motor pins which are automatically mixed from the 4 RC Inputs(throttle,yaw,pitch,roll) into the individual servo1,servo2,servo3,servo4 outputs.

   The full range of Auxiliary Functions can be found [here](https://ardupilot.org/copter/docs/common-auxiliary-functions.html).

*Other Advantages*
Ardupilot tends to be less anal about GPS errors compared to PX4 and Mission Planner also displays the errors too unlike QGC.
Increased number of survey patterns and waypoint options. Spline waypoints provide a smooth path but are not required in missions.
The amount of auxiliary functions(RCx_OPTION) that can be bound in Ardupilot without much additional effort.
Out of the box support for obstacle avoidance with multiple sensors
Support for less sensors. Atm it seems like ardupilot is being updated to support newer sensors more frequently than px4.
More mission types in Ardupilot as compared to QGC

# Mission Planner
Heres a brief overview of mission planner which should aid in setting up basic missions for competitions.

## User Interface and Mission Setup
![logo](https://i.imgur.com/NdZn8ve.png)
Near the left side of mission planner, there is a list of readings that can be changed by double clicking on it. Under Actions tab there are buttons for Takeoff, RTL, shutter, etc

Near the top side middle there is a cockpit. It will display this green blue background by default but can be changed to accept fpv feed from an av to usb receiver or any video source. Any GPS or arming errors will pop up here too.

On the top are the tabs, flight plan is used for planning flight paths and the main configuration is done through initial setup and config and tuning. Simulation is useful for error checking your flight plan to prevent any unforeseen errors or flyaway. **Use it to verify your gps missions before competitions**

![logo](https://imgur.com/u3jqf31.png)

The Flight plan tab has changed through the many revisions, tutorials on youtube use different versions of mission planner but the main differences are the shifting of the polygon tool(to draw search and geofence areas) to the top left and the moving of geofencing to a separate display tab. Geofencing does not seem to work with mission mode but more testing is required to determine this.

The commands will be executed sequentially according to the order given below. The commands are pretty self explanatory and can be searched up easily.

A few notes, DO_DIGICAM_CONTROL activates shutter and DO_CHANGE_SPEED only requires the second term to be filled in to change ground speed.

Always save at least 2 copies of the mission with the buttons on the right to make sure it is not accidentally overwritten and use the load WP and save WP buttons to save a temporary mission that is cleared on reboot.

![logo](https://imgur.com/lsKwuxW.png)

Right clicking on the maps gives you a bunch of options, prefetch can be used to download the map for offline areas.

![logo](https://i.imgur.com/KiN1qAI.png)

Creating multiple waypoints for a survey can be done through the autowp option.

[The full documentation is on the website](https://ardupilot.org/planner/docs/common-planning-a-mission-with-waypoints-and-events.html) but this should provide a basic understanding of what needs to be done for a mission. 

Hitting CTRL-F brings up the temp menu which hides a disgusting amount of buttons with useful features.
![logo](https://i.imgur.com/uAron16.png)
Some of the helpful options for general use are highlighted here.
Mavlink can be used to mirror the stream over a local network, which is useful for opening up another instance of GCS on another computer to observe other parameters.
VLC allows for a usb vtx or another video source to be overlaid.

[This video provides some other useful tips of hidden Mission Planner Features.](https://www.youtube.com/watch?v=yUV8B-9c6d8)




## Misc Tips
BRD_SAFETYENABLE needs to be set to 0 for ESC calibration(else a safety switch needs to be connected),CBRK_IO on newer firmware



# Setting up Autonomous Flight Modes

Below are the required calibrations for autonomous flight. Importantly, tuning the parameters should not be done solely visually, but with the help of the live graphs and the logs. It is important for the flight controller to be able to respond correctly to reach its target setpoint. When flying manually it might not be as apparent that params are incorrect as we have to benefit of correcting visually.

QGC Logs location            |  Mission Planner Logs Location
:-------------------------:|:-------------------------:
![logo](https://i.imgur.com/D4BYmMd.png) |![logo](https://i.imgur.com/xwjKWKZ.png)

Ardupilot logs can be reviewed on the app but for PX4 has this nice website [to review logs](https://logs.px4.io/).

## Mavlink

**fact checking needed** Mavlink is the underlying protocol for communication with the drone. As such, do note that alot of the common errors can be diagnosed via mavlink messages and more advanced functionality such as automation are built upon it. PX4 and Ardupilot both utilise it. With mavlink, sensors can speak to each other and broadcast messages, etc ??? This standard protocol also allows for packages such as Mavros to interface ROS with flight controllers with allow for autonomy related features.

https://mavlink.io/en/about/implementations.html

[Heres a list of mavlink commands](https://dev.px4.io/v1.9.0/en/middleware/modules_command.html) to help with debugging in QGC with PX4(Ardupilot seems to have discontinued it). The listener <device> command is pretty useful to check if a sensor is publishing data(eg listener optical_flow, listener uavcan). Sensors can be manually started and stopped(eg tfmini start, tfmini status, tfmini stop).

## Mavlink Inspector
An important diagnostic tool easily overlooked is the mavlink inspector. This can be used to observe various mavlink values in real-times which is useful for tuning stuff like altitude hold/setpoint. Both GCS have the ability to graph multiple values live which are helpful to validate the changing values.

The full descriptions of [mavlink messages](https://mavlink.io/en/messages/common.html#MAV_CMD) can be found here. Understanding it is essential for autonomous flight modes with ROS or dronekit. Mavlink messages can be sent too in both [Mission Planner](https://ardupilot.org/dev/docs/commonmission-planner-command-line-interface-cli.html) and [QGC](https://docs.qgroundcontrol.com/en/analyze_view/mavlink_console.html) which can carry out any task such as arming or enabling range sensor drivers.

In Ardupilot, Mavlink inspector can be accessed in the temp menu by hitting ctrl-f. Alternatively, quite a lot of readings can be bound to the quick screen in Mission Planner(unlike QGC)
![logo](https://i.imgur.com/Gq7tatI.png)

In PX4, the mavlink inspector is found under the Logs Tab.
![logo](https://i.imgur.com/cUye0sY.png)

The Mavlink port can be broadcasted over a local network which allows you to connect remotely via another computer which is useful for debugging while viewing other sources. Ardupilot has this function built in via the mavlink option on the temp page but this can be carried out on px4 too.

## PID TUNING
Ensure that the PID tuning is stable by checking the response curves for the pitch and roll graphs before attempting to fly in mission/auto flight modes. Overshooting the setpoints will result in oscillations and crashes. It takes time and experience to get the tune right. However, many other factors such as lack of dampening can cause excessive vibrations too, do take note about this before attempting to get a perfect tune.

https://docs.px4.io/v1.9.0/en/config_mc/pid_tuning_guide_multicopter.html
There are 3 sections, go read through the first 2 unless a crash is desired.
https://oscarliang.com/quadcopter-pid-explained-tuning/
[Sample AAVC Test Flight with bad tuning and many oscillations](https://review.px4.io/plot_app?log=b51cbfc0-fb20-46d7-8d30-5252743e3eb2)

### The Ardupilot PID Autotune Shortcut
Ardupilot has this rather convenient feature that allows it to autotune the PID in an open area. Ensure that altitude mode works first before binding the autotune function to a switch. The drone will use step inputs to determine the ideal rates for a sharp response. The AUTOTUNE_AXES param can be changed if your drone does not have enough battery life to tune all axis at once. It is highly recommended to use a GPS as this will allow the drone to maintain position despite windy conditions when autotune is activated from position hold.

PID tuning only starts when both sticks are centered so the throttle will be at 50%, so ensure that altitude hold and hover throttle is tuned properly, more information below.



Here are some very brief notes regarding PID tuning, much more indepth information is available online.

### PID calibration procedure
Unlike racing drones it is impractical for larger, specialized drones to be tuned while flying FPV in acro so a method demanding less of the pilot will be briefly covered below. This process will be easier with one person flying and having a physical feel for how the drone response while the other reads the graphs and changes the params to suit the ideal response and pilots preference.

Firstly, use quick impulse inputs and tune for roll->pitch->yaw one axis at a time. These rapid inputs should only occur for less than a split second as we only want to see how fast it responds to the current input. Repeat the impulse inputs a few times and observe the immediate response to the step input while discarding any external interferences(such as wind or lose parts causing movements). Tuning outdoors is unadvisable(aside from autotune) because of this. Do note that if tuning is conducted midair, always use the 5% increment button to avoid causing big changes in control to the drone and crashing.

The pid graph in PX4 is under Tuning-> Advanced Perimeters. The estimated(Red) is the actual position of the craft while the setpoint(green) is what is desired, therefore the setpoint should hug the estimated as close as possible.

### 1) P, maximum value possible
**_Response_**
Too high - high frequency oscillations(it will be both visible and audible)
Too low - vehicle reacts too slowly

*Increase value until right before it starts oscillating in high frequency* 

### 2) D, minimum value possible
**_Dampening_**
Too high - motors become twitchy and hot(because it overcorrects too quickly, which creates high frequency noise)
Too low - overshoot after step input

*Find Minimum value required so that it corrects quickly(see from graph) even after large step inputs*

### 3) I, balanced
**_Error_** 
Too high - slow oscillations ( as it slowly tries to correct)
Too low - drift, HIGH FREQUENCY OSCILLATION AFTER STEP INPUT

*Increase if drifts in wind/external conditions, decrease if it corrects too slowly/little*
*Recommended values are 0.3-0.5 on PX4 wiki*



Arguably, this is one of the hardest parts of setting up a drone as it requires experience to isolate and identify the actual response.

Isolate - ideally speaking, tuning should be done in an indoor environment without wind but since that is not always possible with bigger drones and lack of space, just be aware that external factors such as wind can cause the response to move on its own. Do not overcompensate for it.

Identify - One common issue is identifying high frequency oscillations from low frequency oscillations when first starting out. The default PIDs from PX4 are deliberately set low for safety but it can be more than doubled on some drones. The biggest tell of high frequency oscillations is that it should be very hard to even take off(unlike low frequency which is still often possible). The drone should both visibly and audibly be constantly vibrating and trying to correct, with little or not control as a result. 

As one might be unable to differentiate high from low frequency oscillations when tuning for the first time, the P tuning might be stopped at a much lower value than what the craft is actually capable of. Considering the size and price of the platform, it is understandable why this is more of an issue as compared to tuning an acro quad. Hence, it maybe useful to do a preliminary tuning of P,D,I and after it is sufficiently stable continue trying to increase P until it is uncontrollable.



The live graphs for PID tuning can be found from the advanced option under the tuning tab.
![logo](https://i.imgur.com/Vz8bNGC.png)
The parameters can be tuned on the fly incrementally.
![logo](https://i.imgur.com/XPvmYo9.png)

### Example tuning process

Here are some images of a tuning process that will perhaps help to judge the feedback of your drone. It is by no means ideal but should aid in helping you to read the graph if you are struggling with identifying what is going on. Do not be overly reliant on the graphs, visual identification and the pilot feedback are as important when tuning.

![logo](https://imgur.com/1Gx4XdM.png)
As previously mentioned, try inputting impulse commands with P and no I or D. In this picture, the response is smaller than the command. Hence, the P has to be increased for increased responsiveness of the drone and to ensure that the graphs are closer to one another. When the drone starts exhibitying high frequency oscillations, either reduce the P **or** start adding in D together with P if more responsiveness is still desired.

![logo](https://imgur.com/GkSbp29.png)
In this graph, the drone oscillates after an impulse input. More D is required to dampen the response quicker.

![logo](https://imgur.com/tvF2A5H.png)
This graph looks pretty okay but the response looks abit slow so the I can be reduced.

![logo](https://imgur.com/ZFJIylE.png)
TBC

## Compass

## Positioning
It should self explanatory why positioning is important for autonomous flight modes. **DO ENSURE THAT ALL THE AUTONOMOUS FLIGHT MODE SPEEDS ARE TURNED DOWN** before attempting first flight.
### Outdoor
As compared to indoor navigation, autonomous outdoor navigation is relatively trivial as it can be accomplished with a gps so long as the drone has view of the sky throughout the mission.
### Indoor
However, if the drone is required to fly in tunnels or buildings where GPS cannot attain lock, other methods to position and localize the drone are needed.

[This is a super handy link in ardupilot wiki](https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html) regarding commonly used sensors for indoor navigation. Amongst those which are used before are the [Intel T265](https://ardupilot.org/copter/docs/common-vio-tracking-camera.html) (which has a in-depth walkthrough in the link) and the [Optical Flow sensors](https://ardupilot.org/copter/docs/common-optical-flow-sensors-landingpage.html) (do ensure to check the optical flow setup link for the full instructions).

The Ardupilot wiki page you mainly mentions optical or beacon based tracking solutions with the exception being cartographer with lidar(at time of writing)

Further details on optical flow configuration are given in the respective Ardupilot and PX4 sections.



## Altitude Hold
An important prerequisite to position hold is the altitude hold. Do note that (IIRC) both Ardupilot and PX4 do not have a param to change the radio controller throttle % at which it occurs and it always occurs when your radio controller is physically at 50% throttle. The settings altered simply changes the value of the throttle pwm value in which the flightstack tries to hover around. Hence, at 50% (+- whatever threshold is set) should allow it to hover and below 50% should activate a velocity based descent depending on how much the sticks are pushed down and above 50% should ascent the drone.

In Ardupilot, [Hover Throttle is automatically learnt](https://ardupilot.org/copter/docs/ac_throttlemid.html#ac-throttlemid) but can still be set manually if desired.

Now that the desired performance is explained, tuning for this axis can occur.
First, attempt to hover the drone while observing the throttle value required in the Mavlink inspector. Set this value as the hover throttle value. (PX4: MPC_THR_HOVER). The params are found under MPC(multicopter position control) in px4.

In PX4, hover throttle can be changed with this slider. There is a param for this value but more testing is needed to determine if the param changes anything else.
![logo](https://i.imgur.com/gHkHTEX.png)

Next, the hover throttle threshold can be set and the throttle gain can tuned to prevent oscillations up and down around hover. (PX4: MPC_Z)

**PLACEHOLDER IMAGE**

***Do note that PX4 has finnicky landing detection in attitude mode, so land in attitude mode, throttle down and switch to stabilized mode to disarm instead of switching modes midair and risking the drone shooting up.***

## Obstacle Avoidance

One important feature of autonomous flight modes is the ability to not crash into obstacles. While rudimentary implementations allow for collision detection, where it [stops infront of objects and multiple obstacle sensors for 360 coverage](https://ardupilot.org/dev/docs/code-overview-object-avoidance.html), increasingly [**ardupilot**](https://ardupilot.org/copter/docs/common-object-avoidance-landing-page.html) and [**px4**](https://docs.px4.io/master/en/computer_vision/collision_prevention.html) have implemented more parameters and algorithms to allow for dynamic path planning so that the drone can move around obstacles. These features are constantly being worked on, so do check the page for updates. In some cases in Ardupilot and [PX4](https://docs.px4.io/master/en/sensor/cm8jl65_ir_distance_sensor.html), specific sensors can be connected directly to the flight controller for obstacle avoidance but(currently) in most of the cases the obstacle avoidance is [done on a companion computer running ROS](https://ardupilot.org/copter/docs/common-realsense-depth-camera.html#common-realsense-depth-camera).

# Indoor Autonomous Flight for Ardupilot(WIP)
Ekf3 must be used instead of ekf2
Homepoint and ekf3 origin must be set on map before each flight, the drone icon should physically show up along with its heading.

Due to the lack of any GPS or markers, the sensor readouts must be rock solid. The [EKF algorithm is also based on the sensor readouts](https://ardupilot.org/copter/docs/common-apm-navigation-extended-kalman-filter-overview.html) so do ensure that there is no interference of the compass, baro, etc. Ensure that the compass does not drift when it is rapidly yawed, [do check this page for more information](https://ardupilot.org/copter/docs/common-magnetic-interference.html). Testing to be done with EMI shielding to make the sensors more stable.

On the topic of sensors and compasses, remember that you can change the orientation of the compass, the **primary compass used** and enable and disable certain compasses. However, [only one compass is used in flight and the other are used for error checking and redundancy](https://ardupilot.org/plane/docs/common-compass-setup-advanced.html#common-compass-setup-advanced).

(Unrelated) You can define which CAN Driver to use under a CAN port, [CAN_P1_DRIVER can be used on either physical port](https://ardupilot.org/copter/docs/parameters.html#can-p1-driver)

# Misc sensor reference for Ardupilot
## ProfiCNC mini carrier board pinout
[Mini carrier board pinout](https://discuss.ardupilot.org/t/pixhawk-2-1-cube-mini-carrier-board-questions/33529/23)
[Alternate link](https://docs.cubepilot.org/user-guides/carrier-boards/mini-carrier-board)


## Obstacle Avoidance Sensors
Most ultrasonic obstacle avoidance sensors comes with an analog value which is scaled based on distance and either a serial or I2C port depending on the version. I2C allows for multiple sensors to be chained together.

Looking at the RNGFNDx pin on the Ardupilot config, either the **ANALOG** can be used via the ADC port or **PWM** can be used when plugged into any of the auxiliary ports. However, due to the restriction of only one ADC port on the pixhawk, only one rangefinder can be used via ADC. The only sensor listed on the documentation capable of PWM OUTPUT is the [Lidarlite](https://ardupilot.org/copter/docs/common-rangefinder-lidarlite.html). This leaves object avoidance with multiple sensors implementation via I2C or Serial. Using this method, since ADC or Analog is not used, the RNGFND param does not have to be changed. 

![logo](https://imgur.com/DcevgxM.png)

Most of the params are self explanatory and configuration can be found on the ardupilot wiki page itself. However, note that [certain libraries such as the Maxbiotix library might not be able to support multiple sensors](https://discuss.ardupilot.org/t/multiple-maxbotix-i2c-pixhawk-2-1-problem/35177/34), more testing is required.

[MaxBiotix MB1200](https://www.maxbotix.com/Ultrasonic_Sensors/MB1200.htm) (I2C Version is MB1202)
The maxbiotix website only shows how to interface a single sensor for terrain following via [analog](https://www.maxbotix.com/articles/ultrasonic-sensors-pixhawk-ardupilot.htm).

The obstacle avoidance can be tested under the proximity button by hitting ctrl-f. [More information here](https://ardupilot.org/dev/docs/code-overview-object-avoidance.html).

## Optical Flow and Lidars

**[READ THIS FIRST]**(https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html), in addition to the sensor specific setup.

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

# PX4 Reference

## Full list of Config

http://docs.px4.io/v1.9.0/en/advanced_config/parameter_reference.html

## Flight Modes
Manual/Stabilized - same for multi rotors, transmit pitch and roll commands to output mixer, output mixer also returns drone to an angle of zero when joysticks are centered.
Acro - transmit angular rates(how fast to go to an angle)
Position - Same as manual/stabilized but tries to correct for wind. May cause crash on larger drones if PID is not tuned properly as will start oscillating. **Exiting Mission mode through max throttle will automatically [change the mode to position mode](https://github.com/PX4/px4_user_guide/blob/master/en/getting_started/flight_modes.md)**

https://dev.px4.io/v1.9.0/en/concept/flight_modes.html




## GPS and EKF
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
IMU drift on warmup. May be resolved by restarting the autopilot. May require an IMU accel and gyro calibration.
-	Adjacent magnetic interference combined with vehicle movement. Resolve my moving vehicle and waiting or re-powering.
-	Bad magnetometer calibration combined with vehicle movement. Resolve by recalibrating.
-	Initial shock or rapid movement on startup that caused a bad inertial nav solution. Resolve by restarting the vehicle and minimizing movement for the first 5 seconds.


### Other Parameters
The following parameters also affect preflight checks.

**COM_ARM_WO_GPS**
The COM_ARM_WO_GPS parameter controls whether or not arming is allowed without a global position estimate.
-	1 (default): Arming is allowed without a position estimate for flight modes that do not require position information (only).
-	0: Arming is allowed only if EKF is providing a global position estimate and EFK GPS quality checks are passing

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

- tfmini start -d /dev/tty/S6
- tfmini status

EKF2_HGT_Mode can be set to range sensor but be sure to test in a safe environment, it might be safer to use baro as the main height sensor.
EKF2_RNG_AID can also be enabled to reduce the barometer inaccuracies caused by ground effect if the range sensor is not the primary sensor.

Use the listener distance_sensor command to test if it still works. A distance_sensor tab should pop up too.

[Heres a useful link for pixhawk fc serial port mapping](https://www.mathworks.com/help/supportpkg/px4/ref/port-mapping-for-serial.html)

### Optical Flow
Before attempting to implement optical flow for the first time, read through [this entire page](https://docs.px4.io/v1.9.0/en/sensor/optical_flow.html). Aside from the obvious stuff like ensuring that the orientation and the offsets of the optical flow sensor is correct, **ensure that a separate downward facing rangefinder is used**(barometer is not allowed) and that integrated_xgyro and integrated_x,.. are the same. For sensors without an integrated imu, integrated_xgyro will show nan and integrated_x will not correspond with HIGHRES_IMU/xgyro.

**If connected to PX4 via (USB) MAVLink the Optical Flow device must publish to the OPTICAL_FLOW_RAD topic, and the distance sensor must publish to the DISANCE_SENSOR topic.** Only then can fusing occur. To view this topic over telem, the param MAV_1_MODE can be changed from normal to onboard to force all the messages to be sent. This can be checked with the command *mavlink status*. Do note that this causes other issues such as inability to pick up impulse inputs on the graph during PID Tuning, so any mavlink driver is set to this during tuning.

Aside from that, diagnosing flow issues are abit harder due to the lack of feedback in PX4. Do check your 1)orientation 2)altitude(needs to be high enough) and 3)xy_vel pids. Even though it might seem like the drone is tracking all of this params could still be wrong. For starters, try reducing the xy_vel_p to 0.08.

#### Hereflow
Take note that on pre 2019 pixhawk 2.1 boards the can 1 and can 2 boards are swapped. The hereflow should be plugged in on the can 1 port by default.

The following links contain the required perimeters for it.
https://discuss.px4.io/t/hereflow-sensor-with-uavcan-not-responding/14694
https://github.com/PX4/px4_user_guide/blob/c03a25be2724e5d89e9ece47425423d5cd100a1d/en/sensor/pmw3901.md
Do note that as of 1.10 the downward rangefinder does not seem to be supported. Hence, a seperate downward facing rangefinder might be needed.

The status of the hereflow can be checked with the mavlink commands ***uavcan status*** and ***listener optical_flow***. Do note that the optical flow mavlink messages are not be broadcasted over telem(inre  PX4) by deafult to save bandwidth.

 [Do note that the x and y axis in the hereflow manual seems to be swapped in px4](https://discuss.cubepilot.org/t/hereflow-setup-instructions-alpha-batch/341) 

Personally, I had issues getting the hereflow to work in the horizontal position so I placed it in the vertical position with the circle on the [pmw3901](https://docs.px4.io/master/en/sensor/pmw3901.html) facing the back.

### Here+/2 GPS
**Considerations**
*These errors show up as generic errors in qgc and is not diagnosable via logs*
- Ideally the base and rover antenna to have clear view of the sky that is 30 degrees above horizon.
- To attain GPS lock look for open spaces with view of sky instead of indoor areas
- **Place GPS away from other electronic devices which generate noise. Eg, FC, other antennas, wifi antennas, power transformers  and anything that emits EM/radio waves.** https://ardupilot.org/copter/docs/common-magnetic-interference.html#common-magnetic-interference
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

https://docs.cubepilot.org/user-guides/here+/here+v2-user-manual

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

https://docs.px4.io/v1.9.0/en/getting_started/led_meanings.html
#### Other Params
 UAVCAN_ENABLE - For GPS via CAN (PX4)

# Misc Tips

In PX4 v1.8.2, calibrating esc does not always change the motor pwm_min or set one that is low enough. Do change it manually if the drone still spins too fast when arming.

In particular, ESCs like the ones on the DJI Snail seems to have issues calibrating with ardupilot and px4. If the other methods in the snail manual do not work, try calibrating each esc manually with a PWM/Servo tester

For reference, at min pwm, motors should not stop spinning when drone is tilted by 60 deg to each side. https://docs.px4.io/v1.9.0/en/config_mc/pid_tuning_guide_multicopter.html

On px4 reflashing to different software versions does not necessarily reset all parameters, ensure reset to factory default is also used

On pixhawk 2.1, battery voltage/current is only read from the first power port, second port will power the device without any readings.
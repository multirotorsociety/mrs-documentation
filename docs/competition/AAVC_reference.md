---
hide:
#  - navigation
---

# AAVC (Autonomous Aerial Vehicle Competition) Reference  
[Facebook for announcements](https://www.facebook.com/groups/1014035222041487/){: .md-button .md-button--primary }
[Website for rules](https://iaai.asia/aavc_Rule.html){: .md-button .md-button--primary }

**Disclaimer from the authors**  
This is a collection of notes taken down by past and present MRS members. Some of the information might be relevant to regular drone competitions that MRS participates in, namely SAFMC and AAVC. This is by no means extensive of what is needed to be known for those competitions nor is it fully fact checked.

## Admin

- Update prof about design and purchasing of items as early as possible
- Inform Crystal (OSL) about overseas trip (once the team lineup is complete), there is alot of admin forms that need to be settled so do this before hand
- To wrap the drone, get bubble wrap from ARMS office

## Misc Rules and tips

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

## Spec'ing
For AAVC 2020 we speced the drone using the weight of the payload and the rough weight of the drone with generic parts. Then, we searched for efficient motors and high density batteries to fit the drone. The propellers and ESC are based on the motor requirements and the previously tested controllers were used for the FC and GPS. However, this might not be the ideal way to do it.

[Ecalc Guide](https://ecalc.ch/calcinclude/help/xcoptercalctutorial.htm){.md-button .md-button--primary}

## Some personal thoughts on frame and drone type in 2019
IMO with motors becoming more powerful and efficient such as the T-Motor U line, at least for a drone with an overall weight of 15kg it makes more sense to use four of these motors in a quad configuration(instead of a hexa or octaquad) to reduce the power draw, complexity and weight. To achieve this efficiency, large propellers have to be used which increases the frame size. This makes CF Rods/Square Tubes an ideal choice of material(due to its accessibility and ease of fabrication) as CF plates are not stiff enough and heavy in larger sheets which makes it impractical to use as a frame the larger it gets. The use of CF Rods in commercial drone with large propellers such as the DJI Matrice and the Freefly Alta X shows that perhaps using CF Rods as arms might be the effective option.

Two of the more common motor designs for the quadrotor class are the x-frame and the h-frame designs. These configurations do not directly define the body of the drones but influence the design of the supporting structures and hence indirectly the weight.
Though there are many variations of motor configurations involving different distances between the motors, IMO the difference is negligible when flying missions as the weight distribution matters more. 

![logo](https://i.imgur.com/UGWarda.jpg)

IMO the x-frame configuration and its variants are harder to use as an test/mission platform over a h-frame/box-frame due to the following reasons.
1) The h-frame/box-frame is more rigid as the body has more connected rods/joints than a x-frame. Trying to remedy this on a x-frame with more rods turns the design into effectively a h-frame. This is important considering the drone has to withstand abuse and crashes. This also ties in to the relatively easier construction/CAD of the h-frame which does not need to have angled parts or a fixed base plate unlike the x frame.
2) The h-frame offers a large mounting area in between the two CF body rods while mounting on a x-frame arguably requires more planning as the base plate will have to be recut (or remoulded) to mount components differently.

Dont take this as META, if you are reading this you probably are just starting out with drones so its okay to try something more tested but experimental drones will teach you more at the end of the day so dont be afraid to expierement.

## Checklists

Drone flying is an activity that has its fair share of troubleshooting required as well as hazards. The following checklists are practises that MRS Members have gained through experience to reduce the amount of troubleshooting required as well as to prevent accidents from happening

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

## Recommended Roles within Teams
Pilot - flies drone/ operates RC transmitter
Backup Pilot - might be required standing beside to the pilot for long range mission where the pilot flies FPV and the backup pilot can take over during LOS
Ground Control - Observes and provides audio feedback to pilot regarding flight status, flight mode,  battery, error states, etc
Spotter - ensures safety of other people and the surrounding, eg weather condition, birds, other drones, intervenes only if there is threat to mission or nearby people
FPV - for spotting objects/mannequin during mission

## Mission Planner vs QGC
There are differences between PX4 and Ardupilot and their target platforms which can be found online. To simplify things for now all you need to know is that Ardupilot is a flight control software in which Mission Planner is the program you download to configure it while PX4 is yet another flight control software in which QGroundControl(QGC) is the program used to configure it. Hardware wise usually the same flight controllers are compatible with both of them but Ardupilot seems to support newer sensors faster.

In AAVC 2020, we used Ardupilot for two main reasons

1. Auto PID Tuning on Ardupilot  
Ardupilot allows for automatic PID Tuning which can be bound to a switch under the config/tuning-> extended tuning page. Param AUTOTUNE_AXES can be set to individually tune one axis at a time so that the battery does not run flat. The drone has to be landed while in PID Mode for the PID to be saved.[Link](https://ardupilot.org/copter/docs/autotune.html)
This saves alot of time normally spent tuning(especially as beginners) as it is hard to tune large drones in SUTD

2. Servo Release without Mixing  
Ardupilot has many presets for mixing the main and auxiliary outputs as unlike PX4 which required mixing for actuation of a servo. By making use of the camera shutter functionality, the payload can be triggered via DIGICAM_CONTROL during missions or bound to a switch. On the pixhawk cube (2), RC8 is Main 8 and RC9 is Aux 1, RC10 is Aux 2, etc... [Link](https://ardupilot.org/copter/docs/common-servo.html). Importantly, the RC ports on the pixhawk can all be set as either input or output. The RCx params control the input behaviour, but when RCx_OPTION is set to 0, the port is set as an an output with the output behaviours being controlled by [the servo params](https://ardupilot.org/copter/docs/common-rcoutput-mapping.html). This includes the 4 motor pins which are automatically mixed from the 4 RC Inputs(throttle,yaw,pitch,roll) into the individual servo1,servo2,servo3,servo4 outputs.

The full range of Auxiliary Functions can be found [here](https://ardupilot.org/copter/docs/common-auxiliary-functions.html).

**Other Advantages**

+ Ardupilot tends to be less anal about GPS errors compared to PX4 and Mission Planner also displays the errors too unlike QGC.

+ Increased number of survey patterns and waypoint options. Spline waypoints provide a smooth path but are not required in missions.

+ The amount of auxiliary functions(RCx_OPTION) that can be bound in Ardupilot without much additional effort.

+ Out of the box support for obstacle avoidance with multiple sensors

+ Support for less sensors. Atm it seems like ardupilot is being updated to support newer sensors more frequently than px4.

+ More mission types in Ardupilot as compared to QGC

## Mission Planner
Heres a brief overview of mission planner which should aid in setting up basic missions for competitions.
Do note that mission based GPS payload drop might not necessarily yield more points than a manual drop(depending on scoring).

### User Interface and Mission Setup
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

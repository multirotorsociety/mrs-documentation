# Introduction to Dronery

This quick start guide should provide you with the basic knowledge needed to understand what is required to setup a basic drone. This **guide will not go into depth about all drone related topics but should provide a basis to allow you to understand how to read online documentation**. As the standards keep changing and manufacturers continue to release new products, **it is vital that you learn how to read online documentation and manuals**.

## Basic Components

![logo](https://imgur.com/AxbRCdi.png)

## Propellers (Props)
Firstly, the things that create lift, the propellers.  
**This should always be put on LAST only after all other checks have been performed. ALWAYS CHECK YOUR SURROUNDINGS AND ASK SOMEONE WITH EXPERIENCE TO CHECK BEFORE PUTTING ON PROPELLERS FOR MAIDEN FLIGHT. The propellers can gravely injure people.** 

<p align="center">
  <img src=https://imgur.com/oabVHUj.png />
  Betaflight default X-frame quadcopter prop configuration
</p>

You would notice that the Motors marked 1 and 4 spin clockwise and the Motors marked 2 and 3 spin anticlockwise. Before placing the propellers, **ensure that the correct CW/CCW propellers are used for the correct motors**. If the motors spin clockwise, the nut should be tightened anticlockwise(and vice versa). This prevents the propellers from loosening during flight.

Different frame configurations will have different motor directions. When in doubt, refer to the documentation on the flightstack(betaflight) itself.

## Power Distribution Board (PDB)
Supplies power from the battery to the motors and converts the battery voltage to 5V for the flight controller, regulates power input to prevent the electronics onboard from damaging levels of power.
The PDB should be orientated such that the big solder pads for the battery terminals should be mounted in the right orientation so that the battery leads can be easily accessed.

<p align="center">
  <img src=https://cdn.getfpv.com/media/catalog/product/cache/1/image/9df78eab33525d08d6e5fb8d27136e95/f/c/fchub-6s_main.jpg />
</p>

Certain PDBs have S1,S2,S3,S4 connections which can be used to wire up motor PWM inputs. In this case, ensure that the S1,S2,S3,S4 corresponds to Motor1,M2,M3,M4. A ribbon cable or wire harness is used to connect the motor inputs to the main flight controller. However the purpose of this is to simplify wiring, there are ports to connect the motor inputs on the flight controller itself.

## Motors
The Motors spin and generate thrust when used with the propellers. They have CW(clockwise) and CCW(Counter clockwise) orientations denoted by the colored screw caps which is threaded in the opposite direction from the rotation to self tighten and prevent propellers from flying out.

![logo](https://us-w1-img-listing.eccang.com/0/dnU2QUpieHE1d1NoMCtOek90MWFZUT09/201808/2767831UYdEoSm8.jpg) 

These motors can spin in the wrong direction, to fix that, either reverse it in firmware (if the ESC supports it) or **swap any of the 2 motor wires**.

## Electronic Speed Control(ESC)
These circuits help to regulate the speed of the motors, they take a PWM signal from the flight controller and combine them with the high power from the PDB/Battery to allow for control of the motor.

For smaller drones, you can find an esc all in one combined with a PDB or even built into the FC but larger drones require a standalone ESC for each motor due to the high power drawn.

All in one ESC Board            |  Standalone ESC
:-------------------------:|:-------------------------:
![](https://cdn.getfpv.com/media/catalog/product/cache/1/image/9df78eab33525d08d6e5fb8d27136e95/k/i/kiss-compactctrl-cc-all-in-one.jpg) |![](https://www.electricwingman.com/img/product/large/TMotorFlame60AESC4.jpg)

On Standalone ESCs, There will be 3 motor wires on one side of the board, and 4/5 wires on the other side of the board. On the other side with 4/5 wires, two of the wires are for battery voltage and battery ground and will be soldered to the PDB. These wires are thicker than the other wires to carry more current. The other 2/3 wires are for signal/ground or signal/power/ground(ignore power for now). The signal should be plugged into the S1/S2/S3/S4 ports respectively and the ground can be plugged into any ground port.

![logo](https://tocircuit.com/wp-content/uploads/2019/11/direct_bb-1024x346.png)

## Batteries (Lipo)
The batteries come with many different chemistries but generally we use the term LiPos(short for Lithium Polymer). A single lipo cell can usually be used from 4.2 to 3.5V. Overdischarge or overcharge results in the battery spoiling. LiPos have a high rate of discharge given by the C rating on the battery which allows the motor to suck more current than regular batteries. 

This is made possible by the low resistance which is essential for the application of drones due to the high current that is drawn by the motors. As a result, a single short across the positive and negative wires is enough to kill the battery and cause an electrical fire due to the low internal resistance unlike AA/AAA batteries. **Special precaution has to be taken when handling lipos** to prevent puncture of cells or overcharge/discharge. 

In terms of charging, the safest way to charge lipos is by 1A per 1000mah(battery capacity). So a 500mah battery would charge at 0.5A. This rate is 1C, different manufacturers can specify different charge and discharge rates for the battery but to preserve battery health a charge rate of 1C is advised.

Lipo chargers allow for charging to HV(4.35V instead of 4.2v) and some do not automatically set a safe current/voltage when charging so do be careful about that.

<p align="center">
  <img src="https://cdn-global-hk.hobbyking.com/media/catalog/product/cache/1/image/9df78eab33525d08d6e5fb8d27136e95/1/7/176399_9067000273-0.jpg" />
</p>  

For long term storage, lipos have to be discharged to its nominal voltage which is usually around 3.7V.  
This is known as **storage charging**.

## Radio Controllers/Transmitter
This transmits a radio signal which is picked up by the receiver on the drone.
<p align="center">
  <img src=http://static1.squarespace.com/static/540b470be4b02405057761f4/5aa35ea68165f533f6f5051e/5aa38c4f08522936111380e4/1575904887238/?format=1500w />
</p>
Radio controllers can have a module bay to add different transmitter modules.

![logo](https://marketchangers.files.wordpress.com/2017/05/dsc7040.jpg?w=930)

Most radio controllers can transmit around 6-12 channels with higher end controllers having up to 18 or more channels. Each channel has an analog value and corresponds to one control such as pitch, yaw or something else. To reduce the need of having one signal and ground wire for each individual channel, protocols are used to transmit multiple channels on a single pin/wire (with 1 ground) such as sbus, ppm, cppm, etc. Documentation on the configuration required can be found by searching individual recievers and radio controllers.

## Receivers
Receivers receive the radio input transmitted by the radio controller.

![logo](https://oscarliang.com/ctt/uploads/2017/08/compact-receiver-options-for-mini-quad-racing-drone-frsky-flysky-tbs-crossfire-futaba-spktrum-r9-2.4ghz-900mhz.jpg)

There are many different transmission protocols made by different manufacturers such as Frsky,Flysky,Futaba,etc. All of which require a compatible transmitter(on the radio) and receiver talking on the same protocol. **Even within a same manufacturer, there are different protocols that are incompatible with each other**!

Configuration can be done on the flight controller software(betaflight), and the radio transmitter and varies on different manufacturers.

## Flight Controller (FC)
The Flight Controller is the brain/CPU of the drone, it controls the drone movements and always contains a usb port for connection to a PC for configuration.

<p align="center">
  <img src=https://imgaz1.staticbg.com/thumb/large/oaupload/banggood/images/38/39/ab3267c9-4273-4051-978b-94e80c7c898b.jpg />
  <br>
  Matek F405 Flight Controller
</p>  

The first thing to notice is the arrow located on the board. **The tip of the arrow represents the front of the drone. The FC should be mounted in the correct orientation**. Conveniently, this board has the pins labelled, but normally this will be indicated in the online manual. The important ports will be explained in greater detail below. **Reading the manual of a FC is essential for understanding in all situations**.

The S1,S2,S3,S4 pins indicate the respective ESC PWM cables that needs to be connected with respect to the pins in the betaflight/flightstack configuration. While depending on the receiver and configuration TX,RX,SBUS pins can be used. 

## Flight Stack
Also known as the Firmware, the Flight Stack is the software which runs on the Flight Controller
There are several popular Flight Stacks out there(some of which are forks of others) but the three main ones that we use are

- [Betaflight](https://github.com/betaflight/betaflight/wiki) (_Used primarily for racing/simple crafts_)
- [PX4](https://px4.io/)
- [Ardupilot](ardupilot.org)

To configure these firmware's, these various software's are used:
- [Betaflight Configurator for Betaflight](https://github.com/betaflight/betaflight-configurator)
- [QGroundControl for PX4](http://qgroundcontrol.com/)
- [Mission Planner for Ardupilot](http://ardupilot.org/)

For simple flying/racing, betaflight is used but for drones that requires additional sensors or intelligence, PX4 or Ardupilot is used. **We will be using betaflight for this workshop**. The list of supported boards of each firmware is contionusly being updated on their individual websites.

[go to top](#top)

## Connections (WIP)
<p align="center">
  <img src=https://www.picclickimg.com/d/l400/pict/271827335392_/1-pair-XT60-male-female-10CM-Silicon-Wire.jpg />
</p>

In order of power coming out from battery. two battery connector wires(positive and negative) are connected to the PDB.

**ENSURE BATTERY POLARITY IS CORRECT BEFORE PLUGGING IN**

<p align="center">
  <img src=http://www.rcdronegood.com/wp-content/uploads/2016/08/Naze-32-Connection-Diagram.jpg />
</p>

The PDB is then connected to the four motors each with 3 wires (in a all in one ESC board). In a standalone PDB, the power (+/-) leads are connected individually to the PDB and the motor PWM inputs are connected either directly to the Flight controller.
The Flight Controller requires connection from the 5V/Ground and the PWM motor inputs minimally. 
The Receiver is connected to the Flight Controller.

And thats it!!! With this connections and after configuration your drone should be able to fly.

## Betaflight configuration

WIP

### ESC calibration

TBA

### Radio calibration

TBA

## Supplementary

### Video Camera
All the camera requires is 5v and ground and it will broadcast a 5.8Ghz signal. Certain flight controllers can output video signal which overlays information such as battery level. There are different channels and bands ensure that multiple video feeds can be broadcasted at the same time on different frequencies.

![logo](https://learnassets.getfpv.com/learn/wp-content/uploads/2019/09/19212447/frequencyChart.png)

### PID Tuning and Rates
PID tuning is a complicated process that requires knowledge on how the drone flies. The tuning process will not be covered in this document as the workshop drone should fly fine with the default PID. [Many resources regarding this topic is avaliable online](https://oscarliang.com/quadcopter-pid-explained-tuning/) and [betaflight recommends that you tune it in acro](https://github.com/betaflight/betaflight/wiki/PID-Tuning-Guide) which is hard given the skill required for it and the space constraints. For now, here is the difference between PID and rates.

![logo](https://imgur.com/0wtD6J1.png)

PID controls the responsiveness and reaction of the quadcopter. This settings are used by the flight controller in all flight modes.

![logo](https://imgur.com/98hpgIr.png)

Rate controls the sensitivity of your drone from your transmitter. Higher rate means that the drone is move sensitive and moves faster while lower means the drone moves slower but gives more resolution on movements. The degrees/s at the end indicate how fast the drone can turn at full stick deflection in a given second. It is important to note that this rates do not control how to flight controller handles the drone but rather how sensitive your inputs on your radio transmitter are. 

Rate profile are given in curves, which allows for lower sensitivity and more precise movements in the middle but the higher sensitivity at the end allows for the pilot to do large maneuvers when the sticks are deflected to the end.

## Some videos for more advanced or indepth topics if you would like to know more
[Electronic Speed Controllers (ESCs) and how they drive Brushless Motors](https://www.youtube.com/watch?v=-mLuU1Nscu4)

[Understanding PID Control](https://www.youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y)

[Drone Control Dynamics 1](https://www.youtube.com/watch?v=hGcGPUqB67Q)

[Drone Control Dynamics 2](https://www.youtube.com/watch?v=GK1t8YIvGM8)

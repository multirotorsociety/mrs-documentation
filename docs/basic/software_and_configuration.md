---
hide:
  - navigation
---

# Software and Configuration

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

## Betaflight configuration

WIP

### ESC calibration

TBA

### Radio calibration

TBA

## Supplementary

### PID Tuning and Rates
PID tuning is a complicated process that requires knowledge on how the drone flies. The tuning process will not be covered in this document as the workshop drone should fly fine with the default PID. [Many resources regarding this topic is avaliable online](https://oscarliang.com/quadcopter-pid-explained-tuning/) and [betaflight recommends that you tune it in acro](https://github.com/betaflight/betaflight/wiki/PID-Tuning-Guide) which is hard given the skill required for it and the space constraints. For now, here is the difference between PID and rates.

![logo](https://imgur.com/0wtD6J1.png)

PID controls the responsiveness and reaction of the quadcopter. This settings are used by the flight controller in all flight modes.

![logo](https://imgur.com/98hpgIr.png)

Rate controls the sensitivity of your drone from your transmitter. Higher rate means that the drone is move sensitive and moves faster while lower means the drone moves slower but gives more resolution on movements. The degrees/s at the end indicate how fast the drone can turn at full stick deflection in a given second. It is important to note that this rates do not control how to flight controller handles the drone but rather how sensitive your inputs on your radio transmitter are. 

Rate profile are given in curves, which allows for lower sensitivity and more precise movements in the middle but the higher sensitivity at the end allows for the pilot to do large maneuvers when the sticks are deflected to the end.

---
hide:
#  - navigation
---

# Ardupilot Info and Documentation

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
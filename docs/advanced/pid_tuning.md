---
hide:
#  - navigation
---

# PID TUNING
Ensure that the PID tuning is stable by checking the response curves for the pitch and roll graphs before attempting to fly in mission/auto flight modes. Overshooting the setpoints will result in oscillations and crashes. It takes time and experience to get the tune right.
**However**, many other factors such as lack of dampening can cause excessive vibrations too, similarly loose screws or components that are not fastened down might generate issues. Visually inspect your drone and check for possible issues that might cause instability before attempting to get a perfect tune.

[PX4 PID Tuning Guide for Multicopters](https://docs.px4.io/v1.9.0/en/config_mc/pid_tuning_guide_multicopter.html)
There are 3 sections, do read through the first 2(even briefly) unless a crash is desired.
[PID Tuning Explained by Oscarliang](https://oscarliang.com/quadcopter-pid-explained-tuning/)
[Sample AAVC Test Flight with bad tuning and many oscillations](https://review.px4.io/plot_app?log=b51cbfc0-fb20-46d7-8d30-5252743e3eb2)

## The Ardupilot PID Autotune Shortcut
Ardupilot has this rather convenient feature that allows it to autotune the PID in an open area. Ensure that altitude mode works first before binding the autotune function to a switch. The drone will use step inputs to determine the ideal rates for a sharp response. The AUTOTUNE_AXES param can be changed if your drone does not have enough battery life to tune all axis at once. It is **highly recommended to use a GPS** as this will allow the drone to maintain position despite windy conditions when autotune is activated from position hold.

PID tuning only starts when both sticks are centered (ie the throttle will be at 50%), so ensure that altitude hold is tuned properly, more information below.

Here are some very brief notes regarding PID tuning, much more indepth information is available online.

## PID calibration procedure
Unlike racing drones it is impractical for larger, specialized drones to be tuned while flying FPV in acro so a method demanding less of the pilot will be briefly covered below. This process will be easier with one person flying and having a physical feel for how the drone response while the other reads the graphs and changes the params to suit the ideal response and pilots preference.

Firstly, use quick impulse inputs and tune for roll->pitch->yaw one axis at a time. These rapid inputs should only occur for less than a split second as we only want to see how fast it responds to the current input(ie inputting a impulse roll right input should cause the drone to momentarily tilt right. If it does not respond, the p is too low. If it starts to move right, then the roll input is too long and needs to be shorter.).

Repeat the impulse inputs a few times and observe the immediate response to the step input while discarding any external interferences(such as wind or lose parts causing movements). Tuning outdoors is unadvisable(aside from autotune) because of this. Do note that if tuning is conducted midair, always use the 5% increment button to avoid causing big changes in control to the drone and crashing.

For a balanced x-frame, you can expect the roll and pitch pids to be roughly the same, which speeds up tuning. For a H frame with the weight distributed along the y axis of the craft, the roll p-pid should be lower than the pitch p-pid as it is easier to roll. 

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

As one might be unable to differentiate high from low frequency oscillations when tuning for the first time, the P tuning might be stopped at a much lower value than what the craft is actually capable of. Considering the size and price of the platform, it is understandable why this is more of an issue as compared to tuning an acro quad. Hence, it maybe useful to do a preliminary tuning of P,D,I and after it is sufficiently stable **continue trying to increase P** until it is uncontrollable.

The live graphs for PID tuning can be found from the advanced option under the tuning tab.
![logo](https://i.imgur.com/Vz8bNGC.png)
The parameters can be tuned on the fly incrementally.
![logo](https://i.imgur.com/XPvmYo9.png)

## Example tuning process

Here are some images of a tuning process that will perhaps help to judge the feedback of your drone. It is by no means ideal but should aid in helping you to read the graph if you are struggling with identifying what is going on. Do not be overly reliant on the graphs, visual identification and the pilot feedback are as important when tuning.

![logo](https://imgur.com/1Gx4XdM.png)
As previously mentioned, try inputting impulse commands with P and no I or D. In this picture, the response is smaller than the command. Hence, the P has to be increased for increased responsiveness of the drone and to ensure that the graphs are closer to one another. When the drone starts exhibiting high frequency oscillations, either reduce the P **or** start adding in D together with P if more responsiveness is still desired.

![logo](https://imgur.com/GkSbp29.png)
In this graph, the drone oscillates after an impulse input. More D is required to dampen the response quicker.

![logo](https://imgur.com/tvF2A5H.png)
This graph looks pretty okay but the response looks abit slow so the I can be reduced.

![logo](https://imgur.com/ZFJIylE.png)
TBC
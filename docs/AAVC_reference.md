# AAVC (Autonomous Aerial Vehicle Competition) Reference  
[Facebook for announcements](https://www.facebook.com/groups/1014035222041487/){: .md-button .md-button--primary }
[Website for rules](https://iaai.asia/aavc_Rule.html){: .md-button .md-button--primary }

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

## Speccing
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
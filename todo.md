# In Sim
(Everett and Trevor, mostly ignore this. I can work on these during flights or at home)
add shooter multiplier button cause calibration is gonna be shit
Make sure things are isolatable (specifically autoDrive and Navigation)
think about roborio2 (they said 30% faster)

# With robot
- shooter velocity control
    - PIDF (getting the robot up to speed with
    - feedforward (set pid controls using kP, kI, and kD in motor then call addFeedforward())
    - statespace (use the loop instantiated in Shooter)
    - ff with voltage drop predictor (try using a normal FF but increasing speed when voltage drops. This indicates ball is passing through and increasing power.
- turret recalibration
    - PhotonVision update
- Shooter test values
    - find a place to shoot (Assembly, outside, gym, etc)
    - mark every foot and drag the robto to that distance (make sure the limelight is reading the distance you are marking)
    - find manually adjust shot til it goes in consistently (i recommend rewriting the FlywheelTest and binding it to button)
- Auto paths
   - I have made a bunch of these
   - to make new ones look into pathweaver and put pathes together in routines.
        - *note* Use the Shot path if you want the robot to stop and shot its gathered balles
   - add your own and test the ones I bs'ed with no experience
- setup shuffleboard
   - show important values
        - turret position
        - turret error
        - flywheel vel
   - choosers
        - auto path
        - control system

# Fancy shooter things:
- movement compensation for targeting (states)
    - this would require better integration between turret and shooter
- make sure to calculate output velocity (based on time of flight) during calibration
    - for now can estimate time of flight as constant
    - iteratively figure out what is should be
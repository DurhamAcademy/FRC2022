# In Sim
- turret sim not be trash

# With robot
- turret recalibration
- test angle controls
- Shooter test values
    - use Trevor spreadsheet (see Software channel)
- Auto paths
  - mess with filesystem

# Fancy shooter things:
- nicer shooter velocity control
    - PIDF (getting the robot up to speed with native)
    - statespace (use the loop instantiated in Shooter)
    - ff with voltage drop predictor (try using a normal FF but increasing speed when voltage drops. This indicates ball is passing through and increasing power.
- movement compensation for targeting (states)
    - this would require better integration between turret and shooter
- make sure to calculate output velocity (based on time of flight) during calibration
    - for now can estimate time of flight as constant
    - iteratively figure out what is should be


## Long Term
- Make sure things are isolatable (specifically autoDrive and Navigation)
- think about roborio2 (they said 30% faster)
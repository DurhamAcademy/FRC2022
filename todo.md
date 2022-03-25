# In Sim

# Second Regionals
- fast spinup (shouldn't take long)
- climb code test (~1 hour)
  - manual climb as well
  - check position controls
  - prepare (measure)
- look at lowering debounces
  - shooter ready
  - turret found
  - AutoShot()
- flash second limelight (try delegating)
  - driverstation image probaly needs to be updated first
- disposal testing
  - ~50% chance this works
- time of flywheel table (~15 minutes)
- shooting while moving

# With robot
- test pose estimator
- test other control schemes
  - inversion
  - native
  - full throttle
  - statespace
    - check how to use faster loop time
- flash other limelight
  - tune ball detection
- test other turret controls
  - velocity 
  - state space
- air time lookup table (would have to be recalibrated with shooter each time)
- test disposal

## random cool things
- Auto Drive to Climb
- Auto Climb
- acceleration calculations

## Full Auto
- goal system (shouldn't be too hard - should update) -> moderate difficulty
  - limelight
- obstacles (can work on this) -> easy, just time intensive
- ball detection in hopper
  - figure out colors senrsors (talk to bots on wheels) -> with resourses, not that hard
- ability to reaccesss if lost for too long -> simple
- robot avoidance -> hard (try installing photon on picam or something)
- climb -> unknown, probably not bad but will need empirics
  - should add endgameInit or something (idk)

## Long Term
- Make sure things are isolatable (specifically autoDrive and Navigation)
- think about roborio2 (they said 30% faster)
- probably get falcons
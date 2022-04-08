# In Sim

- adjust climb arms to be same by default
    - fix issues with manual
- time delay on arm lift so turret safe
- reformat Shuffleboard (require driver-station)
- augmented loop (pg 109 of textbook)
    - detla U: add voltage into state model
        - https://www.chiefdelphi.com/t/971s-control-system/137365/15?u=tatestaples
- https://www.chiefdelphi.com/t/what-impressive-things-did-you-do-in-software-this-year/382245/48?u=tatestaples
- https://github.com/Team254/FRC-2020-Public/blob/master/src/main/java/com/team254/frc2020/planners/DriveMotionPlanner.java#L225
- current management

# With robo
- test other turret controls (it they want to redo tether, than recharacterize) (2 hours) **
    - velocity
    - state space (hard with bad ff)
    -
- test shot while moving (6 hour)
- test why auto shot slow (3 hour)
    - generally make auto better
- flash other limelight (try) (4 hour)
    - tune ball detection
- climb prepare (30 min) [Wednesday]
  - closed loop climb testing
- remove drivetrain swap
- climb stablization (30 min)
- Chris driver practice (whatever time remains) [Thursday]

## random cool things
- Auto Drive to Climb
- Auto Climb

## Full Auto
- goal system (shouldn't be too hard - should update) -> moderate difficulty
    - limelight
- obstacles (can work on this) -> easy, just time intensive
- ball detection in hopper
    - figure out colors senrsors (talk to bots on wheels) -> with resourses, not that hard
    - pypico + 2 rev color sensors
- ability to reaccesss if lost for too long -> simple
- robot avoidance -> hard (try installing photon on picam or something)
- climb -> unknown, probably not bad but will need empirics
    - should add endgameInit or something (idk)

## Long Term

- Make sure things are isolatable (specifically autoDrive and Navigation)
- think about roborio2 (they said 30% faster)
- probably get falcons

### software pitch (11k lines of code written)
- Auto
  - pathweaver option
    - simulation testing (how it works)
  - playback feature
  - Auto pathing
    - obstacle avoidance
    - mention semester independent project (automation as my passion)
  - auto turret
  - auto climb
  - early prototypes of auto shot and intake
- Driver Simplicity
  - custom driver controls (maybe cut)
  - intake retraction (be breif)
    - no penalties
  - shoot from anywhere
    - auto turret
      - limelight
      - pose estimator
      - control
        - wire management
        - side spin
        - back spin
- Additional features (things that really put us apart)
  - driving
    - driver camera
    - continually updating cameras
  - shooting
    - shooting while moving
    - disposal
  - climbing
    - prepare climb
    - auto climb
  - bonus
    - manual overrides
      - every motor
      - simplicity if errors
- Kyber (8.5k lines of code - 75% of all code can be immediately reused)
    - motivation
        - personal journey in understanding
        - documentation for future year
          - I am a senior
    - motor control systems integration
      - purpose
        - rapid iteration
          - can write a bunch of different control schemes and swap them out to see what worked
            - had 6 different control schemes for flywheel at one point
          - single line of code change to switch motors
          - allow for simple simulation support
        - personal learning
        - usability for new coders
      - modes
        - percent
        - position
        - velocity
        - follow
        - current
        - torque
        - music!!
      - control types - each able to be setup in a single line
          - native controls
          - pid
          - ff (intuitive to use)
          - statespace
              - positon, velocity, dual, arm, elevator, drivetrain
              - custom loops with latency compensation
          - bang bang
          - take half back
          - custom
    - Simulation support
        - why important for our robot (time)
            - 3 days of testing
        - easy way to write code for subsystems not there
            - simulated ESC
            - fake solenoids
            - halucinating climb past 2 events


## Extended Pitch
- unit libraries **
  - greater understanding and readiblity
  - custom polar coordinates
  - nav updates
  - turret adjustments
- Characterization
  - use our own code to characterize systems and fit the data
  - allows for more control of the data we get and ensures safety on constrained systems
- Expanded Trajectory generaion
  - obstacles
    - Informed Rapidly Expanding Random Trees
      - illustraion
      - optimizations
    - Traveling Salesman
      - brute
      - nearest neighbors
      - branch and bound
      - general graph class
  - goals
    - CommandManager
- Playback System
  - recording inputs
  - overriding the controller to play inputs
- Navigation / Dynamics
  - integrated and simple to use 
  - built in for all drivetrain types (diff, mecanum, swerve)
    - drive chassis
    - drive auto
    - update auto
    - simulate motors
    - calibration
- debug system
  - allows for graphing and understanding of what everything is doing
- fancy controller modifications
  - drone modification
- LED animations
    - 11 distinct animations
    - sim support and simplicity
- math 
  - multiple coordinate systems
    - robot rel
    - hub rel
    - polar
  - threeD geometry
- builtin mechanism (personal learning and example building)
  - drivetrains
  - elevator
  - arm
  - flywheel
- motor control
  - one line fix to swap out Neos (before musics)
  - experimental statespace controls
    - latency compensation
    - delta U control
  - simulating motors
    - Feedforwards
    - linear systems
- simulations
  - What needs simulation?
    - actuators
      - KSolenoid
      - KMotor
      - LEDs
    - sensors
      - PhotonCamera
      - KGyro
  - Practical uses
    - coding in 3 days
    - make sure nothing hits
    - make sure everything moves at the speeds they should
      - linear systems are really important for this because they allow for guessed simulations before getting to touch the actual motor
- tutorials
  - I am a senior
  - setup by step 
    - what is code -> Control Theory
- UcoSLAM
  - the research process
    - talk about other approaches and their limitations
  - the website
    - relation to our auto Intake prototyping
  - how it works
    - mapping the roof


/**
* Definetly
* - east
* - hedgehogs
* - triple strange
*
* Good:
* sequence
* omegabytes
* pitt pirates
* zebracorns
* platypi :(
* hawtimus prime?
* yeti
* bots on wheels
* green hope falcons (5190)
* gear cats
*
* I want
* collect all the data from NCFIRST matches
* teach me all of the programming required for FRC
* predict all the matches for States
* predict potential from top teams
* graph trends for all the teams over matches
* predict our potential
  */

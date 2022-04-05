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


# With robo (19 hours)

- test other turret controls (it they want to redo tether, than recharacterize) (2 hours) [Monday]
    - state space (hard with bad ff)
        - works well without vel comp
            - try time constant vel comp
        - graph diff heading results
    - *** this needs to be really good to do shoot while move
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

### software pitch

- Features
    - intake
        - no penalties
        - auto intake?
        - driver camera
    - drivetrain
        - fancy odometry
        - auto drive
        - playback controls
        - multiple coordinate systems
            - standard
            - polar
        - speed systems
            - robo centric
            - hub centric
            - field centric
        - lead swap
    - turret
        - statespace
        - limelight
        - working while unplugged
        - self wrapping (protect the cables)
        - side spin comp
        - shoot while moving
        - disposal
    - Shooter
        - conveyor management
        - control schemes
        - falcons
        - shoot while move
        - closed loop, feed when ready
        - active hood
        - speed based on poly fit
    - Climb
        - position controlled
        - prepare for faster climb
        - reaction wheel stablization
    - Controls
        - Modular control system
        - dashboard override on everything
        - turn off fancy things if no work
            - increase flywheel speed if shooting low
            - debug intake snapping
            - force turret out of weird positions
- Kyber
    - motivation
        - personal journey in understanding
        - documentation for future year
        - mechansims
    - motor control systems integration (abstraction allows for modularity)
        - native
        - pid
        - ff (intuitive to use)
        - statespace
            - positon, velocity, dual, arm, elevator, drivetrain
            - custom loops with latency compensation
        - bang
        - custom
    - Characterization Tool
        - use our own code to characterize systems and fit the data
        - allows for more control of the data we get and ensures safety on constrained systems
    - Auto pathing generation
        - shuffleboard integration
        - dynamic path planning
        - communication with other teams
        - role in navigation
        - Playback
            - records all the drivers inputs and plays them back
            - allows for quick and easy non-standard autos
    - debug system
        - allows for graphing and understanding of what everything is doing
    - fancy controller modifications
        - drone modification
        - able to be simulations (see playback)
    - LED animations
    - unit libraries
        - greater understanding and readiblity
        - custom polar coordinates
            - nav updates
            - turret adjustments
    - Pre-built mechanisms
        - personal learning
        - building for the future team
    - Simulation support
        - why important for our robot (time)
            - 2 days of testing
        - branches
            - allow for testing cool new ideas
        - auto testing
            - All autos have actually been developed in sim and debugged on the field
        - maximizing efficiency
            - time with robot is limited
        - increase usability
        - easy way to write code for subsystems not there
            - simulated ESC
            - fake solenoids
            - halucinating climb past 2 events



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

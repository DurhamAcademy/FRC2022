# Motor Control in Kyberlib

This is the kyberlib motor control package. 
It's purpose is to take full advantage of the flexibility of Kotlin to make motor control as intuitive as possible.
Every motor implements Debug and Sendable to be as easy to debug as possible

## KBasicMotorController

This is the base class for any motor. 
### Control Types
It allows for control based on volage to be applied and percent of max output.

It also allows for following another motor. This means that it will copy another motors voltages

### Implementations
All motors in kyberlib include these features but one of the raw BasicMotorControllers is the CTRE KVictorSPX.
Another example is KSpeedController which wraps any WpiLib SpeedController (now MotorController)

## KMotorController
This is the more advanced motor controller. 
This takes full advantage of the abilities of encoders to control both position and velocity in a variety of ways.

### Linear vs Rotation
There are 2 types of motion we actuate, linear and rotation.
In order to control based on linear position and velocities, you must specify a wheel radius so that the class can convert.
If you fail to do so, you will get a LinearUnconfigured Exception

### Gear Ratios and Position Conversion
Another important feature of this class is built-in mechanical advantage calculations. 
Specify the gear ratio or other type converstion factor from input to output position and then operate in the frame of reference that you want!

### Natives
Many encoders have built-in motor control. If you don't specify your own control methods it will default to a built-in method. 
If you see a function with "native" in it, that means it is talking to the lower-level hardware

### Position
One way you can control the motor is based on position. Specify a certain Angle or Length for the motor to go to and it will adjust to be there.

Some important variables for this are:

- position: Angle = the angle of the arm (setter calls positionSetpoint)
- linearPosition: Length = the length travelled (setter calls linearPositionSetpoint)
- positionSetpoint: Angle = angle the motor is trying to get to (get only)
- linearPositionSetpoint: Length = distance the motor is trying to get to (get only)

### Velocity

Another way you can control the motor is based on velocity. Specify a fast you want the motor to go with AngularVelocity or LinearVelocity.

Some important variables for this are:

- velocity: AngularVelocity = the angle of the arm (setter calls positionSetpoint)
- linearVelocity: LinearVelocity = the linear speed of the motor system (setter calls linearVelocitySetpoint)
- velocitySetpoint: AngularVelocity = how fast the motor is trying to spin
- linearVelocitySetpoint: LinearVelocity = how fast the motor is trying to go 

### PID
All KMotorControllers have built in PID Controllers. Specify with kP, kI, and kD. These values are used for calculations and are also passed on native controllers

### Feedforwards
Using motor.addFeedforward(ff), you can automatically incorporate most feedforwards into your position and velocity control.

### Custom Control
When you have a complicated system that you want to create your own control scheme for, override your motors "customControl" variable.

customControl is a suppliable function that provides the coder with the motor as an argument (to retrieve important values) and will apply any voltage return from the function


### Sim Support
You can model any motor in simulation by calling the simUpdate function. 
Provide a feedforward model and the motor will use that to update its positions and velocities
package frc.robot.subsystems

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.robot.Constants

/**
 * Mechanism representing the actuators for climbing
 */
object Climber : SubsystemBase() {

    /** (left) winches that pull the robot up */
    val leftWinch = KSparkMax(40).apply {
        identifier = "left winch"
        radius = Constants.WINCH_RADIUS
        brakeMode = true
        gearRatio = Constants.WINCH_GEAR_RATIO
        motorType = DCMotor.getNeo550(1)
        minLinearPosition = 0.inches
        maxLinearPosition = 24.inches
        currentLimit = 30

        // todo
        kP = 15.0
        kD = 5.0
        setupSim(elevatorSystem(Constants.ROBOT_WEIGHT))
    }

    /** (right) winches that pull the robot up */
    val rightWinch = KSparkMax(41).apply {
        copyConfig(leftWinch)
        identifier = "right winch"
    }

    val highGrab = KSparkMax(-1)  // todo

    inline var extension  // public variable to control position off both the arms
        get() = leftWinch.linearPosition
        set(value) {
            leftWinch.linearPosition = value
            rightWinch.linearPosition = value
        }
}
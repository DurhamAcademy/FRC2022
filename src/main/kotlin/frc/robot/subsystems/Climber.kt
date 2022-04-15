package frc.robot.subsystems

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.math.filters.Differentiator
import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.pneumatics.KSolenoid
import frc.robot.Constants
import frc.robot.RobotContainer

/**
 * Mechanism representing the actuators for climbing
 */
object Climber : SubsystemBase() {
    // pneumatics that lift the climb arms
    private val climbLift = KSolenoid(6, 7, fake = false)
    private val climbLiftFollower = KSolenoid(1, 0, fake = false).apply {
        follow(climbLift)
    }

    private val armSafety = Debouncer(1.0, Debouncer.DebounceType.kFalling)
    var armsLifted
        get() = armSafety.calculate(climbLift.extended)
        set(value) {
            climbLift.extended = armSafety.calculate(value)
        }

    /** (left) winches that pull the robot up */
    val leftWinch = KSparkMax(40).apply {
        identifier = "left winch"
        brakeMode = true
        currentLimit = 30
    }

    /** (right) winches that pull the robot up */
    val rightWinch = KSparkMax(41).apply {
        copyConfig(leftWinch)
        identifier = "right winch"
    }
}
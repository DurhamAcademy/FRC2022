package frc.team6502.robot.subsystems

import edu.wpi.first.wpilibj.simulation.Mechanism2D
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team6502.robot.Constants
import kyberlib.command.Debug
import kyberlib.math.units.extensions.inches
import kyberlib.motorcontrol.KMotorController
import kyberlib.motorcontrol.rev.KSparkMax
import kyberlib.pneumatics.KSolenoid
import kyberlib.simulation.Simulatable
import kotlin.math.absoluteValue

/**
 * Modes for what the climber is doing. Useful for LEDs and other dependencies
 */
enum class CLIMBER_STATUS {
    IDLE, ACTIVE, RISING, FALLING
}

/**
 * Mechanism representing the actuators for climbing
 */
object Climber : SubsystemBase(), Debug, Simulatable {
    var status = CLIMBER_STATUS.IDLE

    // pneumatics that lift the climb arms
    private val leftArmLift = KSolenoid(0, 0)
    private val rightArmLift = KSolenoid(0, 0)

    // winches that pull the robot up
    val leftWinch = KSparkMax(0).apply {
        radius = Constants.WINCH_RADIUS
        brakeMode = true
        gearRatio = Constants.WINCH_GEAR_RATIO
        customControl = { bangBang(it) }
        }
    val rightWinch = KSparkMax(0).apply {
        radius = Constants.WINCH_RADIUS
        brakeMode = true
        gearRatio = Constants.WINCH_GEAR_RATIO
        customControl = { bangBang(it) }
    }

    // public variable to get/set whether the arms are lifted
    var armsLifted
        get() = leftArmLift.extended
        set(value) {
            leftArmLift.extended = value
            rightArmLift.extended = value
        }

    /**
     * Bang bang control used winch position control.
     * If too low, go up hard.
     * If too high, go down hard.
     * @return voltage represented as double
     */
    fun bangBang(motor: KMotorController): Double {
        if(motor.linearPositionError.inches.absoluteValue < 2.0)
            return 0.0
        else if (motor.linearPositionError < 0.inches) return 10.0
        else return -0.0
    }

    /**
     * In progress simulation of the climb
     */
    private val sim = Mechanism2D()
    override fun simUpdate(dt: Double) {
        sim.setLigamentAngle("extendable", if(armsLifted) 90.0f else 10.0f)
        sim.setLigamentLength("extenable", 30f + leftWinch.linearPosition.inches.toFloat())
    }

    override fun periodic() {
        debugDashboard()
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "Left Winch" to leftWinch,
            "Right Winch" to rightWinch,
            "Arms Raised" to armsLifted
        )
    }
}
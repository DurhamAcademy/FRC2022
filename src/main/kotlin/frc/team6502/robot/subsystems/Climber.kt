package frc.team6502.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team6502.robot.Constants
import kyberlib.command.Debug
import kyberlib.motorcontrol.rev.KSparkMax
import kyberlib.pneumatics.KSolenoid

enum class CLIMBER_STATUS {
    IDLE, ACTIVE, RISING, FALLING
}

object Climber : SubsystemBase(), Debug {
    var status = CLIMBER_STATUS.IDLE
    // todo: add sim support for solenoids
    private val leftArmLift = KSolenoid(0, 0)
    private val rightArmLift = KSolenoid(0, 0)

    val leftWinch = KSparkMax(0).apply {
        brakeMode = true
        gearRatio = Constants.WINCH_GEAR_RATIO
        radius = Constants.WINCH_RADIUS
    }
    val rightWinch = KSparkMax(0).apply {
        brakeMode = true
        gearRatio = Constants.WINCH_GEAR_RATIO
        radius = Constants.WINCH_RADIUS
    }

    var armsLifted
        get() = leftArmLift.extended
        set(value) {
            leftArmLift.extended = value
            rightArmLift.extended = value
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
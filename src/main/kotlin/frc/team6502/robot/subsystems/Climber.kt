package frc.team6502.robot.subsystems

import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team6502.robot.Constants
import kyberlib.command.Debug
import kyberlib.motorcontrol.rev.KSparkMax

object Climber : SubsystemBase(), Debug {
    private val leftArmLift = Solenoid(0, 0)
    private val rightArmLift = Solenoid(0, 0)

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
        get() = leftArmLift.get()
        set(value) {
            leftArmLift.set(value)
            rightArmLift.set(value)
        }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "Left Winch" to leftWinch,
            "Right Winch" to rightWinch,
            "Arms Raised" to armsLifted
        )
    }
}
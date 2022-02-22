package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Intaker
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugLevel
import frc.kyberlib.math.units.extensions.radiansPerSecond

/**
 * Deploys and activates the intake
 */
object Intake : CommandBase() {
    init {
        addRequirements(Intaker)
    }

    override fun initialize() {
        Debug.log("Intake", "start", level=DebugLevel.LowPriority)
//        Intaker.deployed = true
        Intaker.intakeMotor.percent = Constants.INTAKE_PERCENT
    }

    override fun end(interrupted: Boolean) {
        Debug.log("Intake", "end", level=DebugLevel.LowPriority)
//        Intaker.deployed = false
        Intaker.intakeMotor.stop()
    }
}
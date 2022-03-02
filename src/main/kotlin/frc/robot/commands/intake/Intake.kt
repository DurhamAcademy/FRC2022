package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugLevel

//hi
import frc.robot.subsystems.Intaker
import frc.robot.subsystems.Conveyor

/**
 * Deploys and activates the intake
 */
object Intake : CommandBase() {
    init {
        addRequirements(Intaker)
    }

    override fun initialize() {
        Debug.log("Intake", "start", level=DebugLevel.LowPriority)
        Intaker.intakeMotor.percent = Constants.INTAKE_PERCENT
        Intaker.deployed = true
    }

    override fun end(interrupted: Boolean) {
        Debug.log("Intake", "end", level=DebugLevel.LowPriority)
        Intaker.deployed = false
        Intaker.intakeMotor.stop()
    }
}

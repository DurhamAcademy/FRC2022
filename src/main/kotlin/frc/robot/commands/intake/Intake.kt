package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.robot.subsystems.Intaker

/**
 * Deploys and activates the intake
 */
object Intake : CommandBase() {
    init {
        addRequirements(Intaker)
    }

    override fun initialize() {
        Debug.log("Intake", "start", level=DebugFilter.LowPriority)
        println("intaking")
        Intaker.deployed = true
        Intaker.intakeMotor.percent = Constants.INTAKE_PERCENT
    }

    override fun end(interrupted: Boolean) {
       Debug.log("Intake", "end", level=DebugFilter.LowPriority)
       Intaker.deployed = false
       Intaker.intakeMotor.stop()
    }

    override fun isFinished(): Boolean {
        return false;
    }
}
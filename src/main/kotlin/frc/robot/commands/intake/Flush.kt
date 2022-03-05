package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.robot.Constants
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.Intaker

object Flush : CommandBase() {
    init {
        addRequirements(Intaker, Conveyor)
    }

    override fun initialize() {
        Debug.log("Intake", "start", level= DebugFilter.LowPriority)
        println("intaking")
        Intaker.deployed = true
        Intaker.intakeMotor.percent = -Constants.INTAKE_PERCENT
        Conveyor.conveyor.voltage = -10.0
    }

    override fun end(interrupted: Boolean) {
        Debug.log("Intake", "end", level= DebugFilter.LowPriority)
        Intaker.deployed = false
        Intaker.intakeMotor.stop()
        Conveyor.conveyor.stop()
    }

    override fun isFinished(): Boolean {
        return false;
    }
}
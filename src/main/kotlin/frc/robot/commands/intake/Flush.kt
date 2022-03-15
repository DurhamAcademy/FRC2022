package frc.robot.commands.intake

import edu.wpi.first.wpilibj.Timer
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

    val timer = Timer()

    override fun initialize() {
        Debug.log("Intake", "start", level= DebugFilter.Low)
        timer.reset()
        timer.start()
        Intaker.deployed = true
        Intaker.intakeMotor.percent = -Constants.INTAKE_PERCENT
    }

    override fun execute() {
        Conveyor.conveyor.percent = if(timer.hasElapsed(.1)) -.9 else .2
    }

    override fun end(interrupted: Boolean) {
        Debug.log("Intake", "end", level= DebugFilter.Low)
        Intaker.deployed = false
        Intaker.intakeMotor.stop()
        Conveyor.stop()
    }

    override fun isFinished(): Boolean {
        return false;
    }
}
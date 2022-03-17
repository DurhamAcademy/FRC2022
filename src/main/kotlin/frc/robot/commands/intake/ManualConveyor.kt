package frc.robot.commands.intake

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Conveyor

object ManualConveyor : CommandBase() {
    init {
        addRequirements(Conveyor)
    }

    override fun initialize() {
        SmartDashboard.putNumber("conveyor", 0.0)
        SmartDashboard.putNumber("indexer", 0.0)
    }

    override fun execute() {
        Conveyor.conveyor.percent = SmartDashboard.getNumber("conveyor", 0.0)
        Conveyor.feeder.percent = SmartDashboard.getNumber("indexer", 0.0)
    }

    override fun end(interrupted: Boolean) {
        Conveyor.stop()
    }
}
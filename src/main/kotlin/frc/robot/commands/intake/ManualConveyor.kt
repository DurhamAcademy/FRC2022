package frc.robot.commands.intake

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Conveyor

object ManualConveyor : CommandBase() {
    const val conveyorString = "conveyor percent"
    const val feederString = "feeder percent"
    init {
        addRequirements(Conveyor)
        SmartDashboard.putNumber(conveyorString, 0.0)
        SmartDashboard.putNumber(feederString, 0.0)
    }

    override fun execute() {
        Conveyor.conveyor.percent = SmartDashboard.getNumber(conveyorString, 0.0)
        Conveyor.feeder.percent = SmartDashboard.getNumber(feederString, 0.0)
    }

    override fun end(interrupted: Boolean) {
        Conveyor.stop()
    }
}
package frc.robot.commands.conveyor

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Conveyor

object ManualConveyor : CommandBase() {
    private const val conveyorString = "conveyor percent"
    private const val feederString = "feeder percent"
    init {
        addRequirements(Conveyor)
        SmartDashboard.putNumber(conveyorString, 0.0)
        SmartDashboard.putNumber(feederString, -0.5)
    }

    override fun execute() {
        Conveyor.custom(
            SmartDashboard.getNumber(conveyorString, 0.0),
            SmartDashboard.getNumber(feederString, 0.0)
        )
    }

    override fun end(interrupted: Boolean) {
        Conveyor.stop()
    }
}
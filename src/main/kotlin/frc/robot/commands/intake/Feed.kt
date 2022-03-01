package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.subsystems.ConveyorStatus
import frc.robot.subsystems.Conveyor

object Feed : CommandBase() {
    init {
        addRequirements(Conveyor)
    }

    override fun initialize() {
        Conveyor.status = ConveyorStatus.FEEDING
        Conveyor.conveyor.percent = 0.5
        Conveyor.feeder.percent = 0.9
    }

    override fun isFinished(): Boolean = !RobotContainer.controlScheme.SHOOT.get()
}
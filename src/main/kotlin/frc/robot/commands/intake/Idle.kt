package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ConveyorStatus
import frc.robot.subsystems.Conveyor

object Idle : CommandBase() {  // todo: think about Idle also spinning intake
    init {
        addRequirements(Conveyor)
    }

    override fun initialize() {
        Conveyor.status = ConveyorStatus.IDLE
        Conveyor.feeder.percent = -0.1
        Conveyor.conveyor.percent = -0.1
    }

    override fun end(interrupted: Boolean) {
//        Conveyor.feeder.stop()
//        Conveyor.conveyor.stop()
    }
}
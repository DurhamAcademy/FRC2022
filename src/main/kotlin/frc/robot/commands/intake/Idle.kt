package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Conveyor

object Idle : CommandBase() {
    init {
        addRequirements(Conveyor)
    }

    override fun initialize() {
        Conveyor.feeder.percent = -0.1
        Conveyor.ConveyorMotor.percent = 0.1
    }

    override fun end(interrupted: Boolean) {
//        Conveyor.feeder.stop()
//        Conveyor.ConveyorMotor.stop()
    }
}
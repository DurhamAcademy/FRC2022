package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.Shooter

object Eject : CommandBase() {
    /// TODO: 2/25/22 add Eject Code
    override fun initialize() {
        addRequirements(Shooter)
        addRequirements(Conveyor)
        beforeStarting(LoadEject)
    }

    override fun execute() {
        Conveyor.feeder.voltage = 0.9
        Conveyor.indexer.voltage = 0.35
    }

    override fun end(interrupted: Boolean) {
        Conveyor.feeder.voltage = 0.0
        Conveyor.indexer.voltage = 0.1
    }
}
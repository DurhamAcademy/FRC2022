package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.rotationsPerSecond
import frc.kyberlib.math.units.extensions.rpm
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Turret

object Eject : CommandBase() {
    /// TODO: 2/25/22 add Eject Code
    override fun initialize() {
        addRequirements(Shooter)
        addRequirements(Turret)
        addRequirements(Conveyor)
        beforeStarting(LoadEject)
    }

    override fun execute() {
        Conveyor.feeder.velocity = 5.rotationsPerSecond
        Conveyor.indexer.velocity = 10.rotationsPerSecond
    }

    override fun end(interrupted: Boolean) {
        Conveyor.feeder.velocity = 5.rpm
        Conveyor.indexer.velocity = 3.rpm
    }
}
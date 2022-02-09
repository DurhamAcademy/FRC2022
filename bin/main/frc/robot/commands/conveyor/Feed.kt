package frc.robot.commands.conveyor

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.Shooter


/**
 * Removes ball of wrong color
 */
object Feed : CommandBase() {
    init {
        addRequirements(Conveyor)
    }

    override fun execute() {
        Conveyor.indexer.percent = 0.5
        Conveyor.feeder.percent = 0.5
    }

    override fun isFinished(): Boolean = true  // FIXME: Figure this out
}
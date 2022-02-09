package frc.robot.commands.conveyor

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Conveyor


/**
 * Removes ball of wrong color
 */
object Agitate : CommandBase() {
    init {
        addRequirements(Conveyor)
    }

    override fun execute() {
        Conveyor.feeder.percent = -0.1
        Conveyor.indexer.stop()
    }
}
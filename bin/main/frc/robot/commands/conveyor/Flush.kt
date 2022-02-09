package frc.robot.commands.conveyor

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Intaker
import frc.robot.subsystems.Conveyor


/**
 * Removes ball of wrong color
 */
object Flush : CommandBase() {
    init {
        addRequirements(Intaker, Conveyor)
    }

    override fun execute() {
        Intaker.deployed = true
        Intaker.intakeMotor.percent = -1.0

        Conveyor.indexer.percent = -0.5
    }
}
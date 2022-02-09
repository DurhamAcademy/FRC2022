package frc.robot.commands.conveyor

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.CONVEYOR_STATUS
import frc.robot.subsystems.Shooter
import edu.wpi.first.wpilibj.Timer


/**
 * Removes ball of wrong color
 */
object Feed : CommandBase() {
    init {
        addRequirements(Conveyor)
    }
    
    val timer = Timer()

    override fun initialize() {
        Conveyor.status = CONVEYOR_STATUS.FEEDING
        timer.reset()
        timer.start()
    }

    override fun execute() {
        Conveyor.indexer.percent = 0.5
        Conveyor.feeder.percent = 0.5
    }

    override fun end(interrupted: Boolean) {
        Conveyor.status = CONVEYOR_STATUS.EMPTY
    }

    override fun isFinished(): Boolean = timer.hasElapsed(2.0)  // TODO: make better
}
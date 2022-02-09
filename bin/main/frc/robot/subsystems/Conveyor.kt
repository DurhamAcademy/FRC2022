package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.robot.commands.conveyor.Feed
import frc.robot.commands.conveyor.Agitate

/**
 * State of the hopper. Used for LEDs and other dependencies
 */
public enum class CONVEYOR_STATUS {
    EMPTY, SINGLE_GOOD, FULL_GOOD, BAD
}

/**
 * Controls all aspects of the hopper.
 * Waiting for design to be finalized before code is added
 */
object Conveyor : SubsystemBase() {
    var status = CONVEYOR_STATUS.FULL_GOOD
    
    val indexer = KSparkMax(0)
    val feeder = KSparkMax(0)

    init {
        defaultCommand = Agitate
    }

    val good
        get() = status == CONVEYOR_STATUS.FULL_GOOD || status == CONVEYOR_STATUS.SINGLE_GOOD

    fun feed() {
        Feed.schedule()
    }
}
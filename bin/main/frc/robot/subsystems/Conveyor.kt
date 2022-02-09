package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase

/**
 * State of the hopper. Used for LEDs and other dependencies
 */
enum class CONVEYOR_STATUS {
    EMPTY, SINGLE_GOOD, FULL_GOOD, BAD, FEEDING
}

/**
 * Controls all aspects of the hopper.
 * Waiting for design to be finalized before code is added
 */
object Conveyor : SubsystemBase() {
    var status = CONVEYOR_STATUS.FULL_GOOD
    // idk what this is gonna look like man

    val good
        get() = status == CONVEYOR_STATUS.FULL_GOOD || status == CONVEYOR_STATUS.SINGLE_GOOD

    fun feed() {
        status = CONVEYOR_STATUS.FEEDING
    }
}
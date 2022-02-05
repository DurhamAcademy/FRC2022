package frc.team6502.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase


enum class CONVEYOR_STATUS {
    EMPTY, SINGLE_GOOD, FULL_GOOD, BAD, FEEDING
}


object Conveyor : SubsystemBase() {
    var status = CONVEYOR_STATUS.FULL_GOOD
    // idk what this is gonna look like man

    val hasBalls: Boolean = false // todo

    fun feed() {
        status = CONVEYOR_STATUS.FEEDING
    }
}
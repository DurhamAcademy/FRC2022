package frc.robot.subsystems


/**
 * Current status of the shooter mechanism
 */
enum class SHOOTER_STATUS {
    IDLE, SPINUP, LOW_READY, HIGH_READY, AUTO_SHOT, FORCE_SHOT
}
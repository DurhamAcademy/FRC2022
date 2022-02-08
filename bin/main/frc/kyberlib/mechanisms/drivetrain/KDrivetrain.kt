package frc.kyberlib.mechanisms.drivetrain

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import frc.kyberlib.command.Debug

/**
 * Interface for all pre-made Drivetrains
 */
interface KDrivetrain : Debug {
    /**
     * Moves the chassis to the designated speed
     */
    fun drive(speeds: ChassisSpeeds)

    val chassisSpeeds: ChassisSpeeds
}
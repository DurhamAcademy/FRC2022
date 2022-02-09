package frc.kyberlib.mechanisms.drivetrain.swerve

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.math.units.extensions.k

/**
 * Standard Swerve Module. One motor drives speed and the other controls rotation
 * @param location location of module relative to center of rotation on the Chassis
 * @param driveMotor motor that provides power to the wheel
 * @param turnMotor motor that controls rotation of the Module
 */
open class StandardSwerveModule (location: Translation2d, private val driveMotor: KMotorController, private val turnMotor: KMotorController) : SwerveModule(location) {
    // turn controls
    override var rotation: Rotation2d
        get() = turnMotor.position.normalized
        set(value) {
            turnMotor.position = value.k.normalized
        }

    // drive info
    override var speed
        get() = driveMotor.linearVelocity
        set(value) {
            driveMotor.linearVelocity = value
        }
}

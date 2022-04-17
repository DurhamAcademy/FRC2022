package frc.kyberlib.mechanisms.drivetrain.swerve

import edu.wpi.first.math.geometry.Translation2d
import frc.kyberlib.math.units.extensions.Angle
import frc.kyberlib.math.units.extensions.normalized
import frc.kyberlib.motorcontrol.KMotorController
import kotlin.math.PI

/**
 * Standard Swerve Module. One motor drives speed and the other controls rotation
 * @param location location of module relative to center of rotation on the Chassis
 * @param driveMotor motor that provides power to the wheel
 * @param turnMotor motor that controls rotation of the Module
 */
open class StandardSwerveModule(
    location: Translation2d,
    private val driveMotor: KMotorController,
    private val turnMotor: KMotorController
) : SwerveModule(location) {

    init {
        turnMotor.PID.enableContinuousInput(-PI, PI)
    }
    // turn controls
    override var rotation: Angle
        get() = turnMotor.position
        set(value) {
            turnMotor.position = value
        }

    // drive info
    override var speed
        get() = driveMotor.linearVelocity
        set(value) {
            driveMotor.linearVelocity = value
        }
}

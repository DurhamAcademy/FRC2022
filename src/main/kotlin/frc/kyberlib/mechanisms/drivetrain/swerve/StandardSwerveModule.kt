package frc.kyberlib.mechanisms.drivetrain.swerve

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.kyberlib.math.units.extensions.*
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
    override val rotation: Angle
        get() = turnMotor.position

    // drive info
    override val speed
        get() = driveMotor.linearVelocity

    override var optimizedState: SwerveModuleState
        get() = SwerveModuleState(speed.metersPerSecond, rotation.w)
        set(value) {
            driveMotor.linearVelocity = value.speedMetersPerSecond.metersPerSecond
            turnMotor.position = value.angle.k
        }
}

package frc.kyberlib.mechanisms.drivetrain.swerve

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.kyberlib.command.Debug
import frc.kyberlib.math.kEpsilon
import frc.kyberlib.math.units.extensions.*

/**
 * Class that manages the speed and rotation of a swerve package
 */
abstract class SwerveModule(val location: Translation2d) : Debug {
    /**
     * The rotation that the wheel is facing
     */
    abstract var rotation: Angle  // fixme: this needs to be profiled

    /**
     * The speed that the motor should drive
     */
    abstract var speed: LinearVelocity

    /**
     * Stores the swerveModule state.
     * Optimizes to the nearest angle and normalized speed
     */
    var state: SwerveModuleState
        get() = SwerveModuleState(speed.metersPerSecond, rotation.w)
        set(value) {
            val optimized = SwerveModuleState.optimize(value, rotation.w)
            stateSetpoint = optimized
            speed = optimized.speedMetersPerSecond.metersPerSecond
            rotation = if(optimized.speedMetersPerSecond == 0.0) rotation
                        else optimized.angle.k
        }

    /**
     * The state that the module is going for
     */
    var stateSetpoint: SwerveModuleState = brakeState
        private set

    /**
     * Gets the state that the module should go to in order to lock the robot in place.
     * (Normal to the line between location and center of rotation)
     */
    val brakeState: SwerveModuleState
        get() = SwerveModuleState(0.0, Rotation2d(location.x, location.y).rotateBy(90.degrees.w))

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "Rotation (rad)" to rotation.radians,
            "Speed (m per s)" to speed.metersPerSecond
        )
    }
}
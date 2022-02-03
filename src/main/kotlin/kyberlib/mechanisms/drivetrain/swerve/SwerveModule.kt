package kyberlib.mechanisms.drivetrain.swerve

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import kyberlib.command.Debug
import kyberlib.math.units.extensions.LinearVelocity
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.k
import kyberlib.math.units.extensions.metersPerSecond

/**
 * Class that manages the speed and rotation of a swerve package
 */
abstract class SwerveModule(val location: Translation2d) : Debug {
    /**
     * The rotation that the wheel is facing
     */
    abstract var rotation: Rotation2d

    /**
     * The speed that the motor should drive
     */
    abstract var speed: LinearVelocity

    /**
     * Stores the swerveModule state.
     * Optimizes to the nearest angle and normalized speed
     */
    var state: SwerveModuleState
        get() = SwerveModuleState(speed.metersPerSecond, rotation)
        set(value) {
            val optimized = SwerveModuleState.optimize(value, rotation)
            stateSetpoint = optimized
            rotation = optimized.angle.k
            speed = optimized.speedMetersPerSecond.metersPerSecond
        }

    /**
     * The state that the module is going for
     */
    var stateSetpoint: SwerveModuleState = breakState
        private set

    /**
     * Gets the state that the module should go to in order to lock the robot in place.
     * (Normal to the line between location and center of rotation)
     */
    val breakState: SwerveModuleState
        get() = SwerveModuleState(0.0, Rotation2d(location.x, location.y).rotateBy(90.degrees))

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "Rotation (rad)" to rotation.radians,
            "Speed (m/s)" to speed.metersPerSecond,
            "location" to location
        )
    }
}
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
    abstract val rotation: Angle

    /**
     * The speed that the motor should drive
     */
    abstract val speed: LinearVelocity

    /**
     * Stores the swerveModule state.
     * Optimizes to the nearest angle and normalized speed
     */
    var state: SwerveModuleState
        get() = optimizedState
        set(value) {
            stateSetpoint = optimize(value)
            optimizedState = stateSetpoint
        }

    protected abstract var optimizedState: SwerveModuleState

    private fun optimize(state: SwerveModuleState): SwerveModuleState {
        val opt = SwerveModuleState.optimize(state, rotation.w)
        if(state.speedMetersPerSecond == 0.0) opt.angle = rotation.w
        return opt
    }

    /**
     * The state that the module is going for
     */
    var stateSetpoint: SwerveModuleState = SwerveModuleState()
        protected set

    /**
     * Gets the state that the module should go to in order to lock the robot in place.
     * (Normal to the line between location and center of rotation)
     */
    val brakeState: SwerveModuleState = SwerveModuleState(0.0, Rotation2d(location.x, location.y).rotateBy(90.degrees.w))
}
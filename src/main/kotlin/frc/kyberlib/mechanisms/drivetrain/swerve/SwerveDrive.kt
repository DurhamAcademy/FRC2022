package frc.kyberlib.mechanisms.drivetrain.swerve

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.kyberlib.math.units.debugValues
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.mechanisms.drivetrain.KDrivetrain
import frc.kyberlib.mechanisms.drivetrain.dynamics.KSwerveDynamics

/**
 * Pre-made Swerve Drivetrain.
 * @param gyro provides heading information
 * @param swerveModules are used to physically move the robot
 * @param constraints optional value that regulates how the robot can move
 */
abstract class SwerveDrive : KDrivetrain() {
    abstract override val dynamics: KSwerveDynamics
    open val maxVelocity = 3.metersPerSecond
    /**
     * Drive the robot is given directions
     * @param speeds The speeds to move the robot
     * @param fieldOriented whether to drive relative to the driver or relative to the robot
     */
    fun drive(speeds: ChassisSpeeds, fieldOriented: Boolean = true) {
        dynamics.drive(speeds, fieldOriented)
    }

    /**
     * Individually sets the states for each module
     */
    fun drive(vararg states: SwerveModuleState) {
        dynamics.drive(*states)
    }

    /**
     * Locks the wheels to prevent being pushed
     */
    fun lock() {
        dynamics.stop()
    }

    override fun debugValues(): Map<String, Any?> {
        val map = mutableMapOf<String, Any>(
            "speed" to chassisSpeeds.debugValues,
            "dynamics" to dynamics
        )

        return map.toMap()
    }

}
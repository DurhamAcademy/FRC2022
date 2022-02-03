package kyberlib.mechanisms.drivetrain.swerve

import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand
import kyberlib.command.Debug
import kyberlib.math.units.debugValues
import kyberlib.math.units.extensions.KRotation
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.feetPerSecond
import kyberlib.mechanisms.drivetrain.KDrivetrain
import kyberlib.sensors.gyros.KGyro

/**
 * Pre-made Swerve Drivetrain.
 * @param gyro provides heading information
 * @param swerveModules are used to physically move the robot
 * @param constraints optional value that regulates how the robot can move
 */
class SwerveDrive(private val gyro: KGyro,
                  private vararg val swerveModules: SwerveModule,
                  private val constraints: TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(10.feetPerSecond.value, 10.feetPerSecond.value)
                        ) : SubsystemBase(), KDrivetrain, Debug {
    // controls
    private val kinematics = SwerveDriveKinematics(*swerveModules.map { it.location }.toTypedArray())
    private val odometry = SwerveDriveOdometry(kinematics, 0.degrees)

    // field relative settings
    private var fieldHeading = gyro.heading

    var states
        get() = swerveModules.map { it.state }
        set(value) {swerveModules.zip(value).forEach { it.first.state = it.second }}

    override val chassisSpeeds: ChassisSpeeds
        get() =  kinematics.toChassisSpeeds(*states.toTypedArray())
    override var heading: KRotation
        get() = gyro.heading
        set(value) { gyro.heading = value }
    override var pose: Pose2d
        get() = odometry.poseMeters
        set(value) { odometry.resetPosition(value, heading) }

    override fun drive(speeds: ChassisSpeeds) {
        drive(speeds, true)
    }

    /**
     * Drive the robot is given directions
     * @param speeds The speeds to move the robot
     * @param fieldOriented whether to drive relative to the driver or relative to the robot
     */
    fun drive(speeds: ChassisSpeeds, fieldOriented: Boolean = true) {
        if (fieldOriented) {
            val fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, gyro.heading.minus(fieldHeading))
            drive(*kinematics.toSwerveModuleStates(fieldSpeeds))
        }
        else drive(speeds)
    }

    /**
     * Individually sets the states for each module
     */
    fun drive(vararg states: SwerveModuleState) {
        assert(states.size == swerveModules.size) { "The size of states(${states.size}) do no match the number of modules ${swerveModules.size}" }
        SwerveDriveKinematics.normalizeWheelSpeeds(states, constraints.maxVelocity)
        swerveModules.zip(states).forEach { it.first.state = it.second }
    }

    /**
     * Locks the wheels to prevent being pushed
     */
    fun lock() {
        drive(*swerveModules.map { it.breakState }.toTypedArray())
    }

    /**
     * Called periodically.
     * Updates the robot location with each of the modules states
     */
    override fun periodic() {
        val moduleStates = swerveModules.map { it.state }
        odometry.update(gyro.heading, *moduleStates.toTypedArray())
    }

    val xControl = PIDController(0.7, 0.0, 0.1)
    val yControl = PIDController(0.7, 0.0, 0.1)
    val thetaControl = ProfiledPIDController(0.7, 0.0, 0.1, constraints)

    /**
     * generates a command to follow a trajectory
     */
    fun auto(trajectory: Trajectory): SwerveControllerCommand {
        return SwerveControllerCommand(
            trajectory,
            odometry::getPoseMeters,
            kinematics,
            xControl, yControl, thetaControl,
            this::drive,
            this
        )
    }

    override fun debugValues(): Map<String, Any?> {
        val map = mutableMapOf<String, Any>(
            "pose" to pose.debugValues,
            "speed" to chassisSpeeds.debugValues,
        )
        swerveModules.forEachIndexed { index, swerveModule -> map["swerve Module #$index"] = swerveModule }
        return map.toMap()
    }

}
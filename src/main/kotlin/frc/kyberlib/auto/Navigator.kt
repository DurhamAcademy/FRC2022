package frc.kyberlib.auto

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N5
import frc.kyberlib.auto.trajectory.KTrajectory
import frc.kyberlib.auto.trajectory.KTrajectoryConfig
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.command.LogMode
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.zeroPose
import frc.kyberlib.sensors.gyros.KGyro
import frc.kyberlib.simulation.field.KField2d
import frc.robot.subsystems.Drivetrain

enum class TrackingMode {
    Fast, Fancy, DumbBoth
}

/**
 * Class to store and update robot navigation information
 */
class Navigator(
    val gyro: KGyro,
    startPose: Pose2d = zeroPose,
    private val trackingMode: TrackingMode = TrackingMode.DumbBoth
) : Debug {
    //    override val priority: DebugFilter = DebugFilter.Max
    companion object {
        var instance: Navigator? = null
    }

    init {
        instance = this
//        gyro.heading = startPose.rotation.k
    }

    private val useOdometry = trackingMode != TrackingMode.Fancy

    /**
     * A probability calculator to guess where the robot is from odometer and vision updates
     */
    val poseEstimator = DifferentialDrivePoseEstimator(
        gyro.heading.w, startPose,
        MatBuilder(N5.instance, N1.instance).fill(0.02, 0.02, 0.01, 0.02, 0.02),
        MatBuilder(N3.instance, N1.instance).fill(0.02, 0.01, 0.01),
        MatBuilder(N3.instance, N1.instance).fill(0.1, 0.1, 0.01)
    )

    /**
     * A object with restrictions on how the robot will move
     */
    private var pathingConfig =
        KTrajectoryConfig(5.metersPerSecond, 5.metersPerSecond).apply { KTrajectory.generalConfig = this }

    fun applyMovementRestrictions(velocity: LinearVelocity, maxAcceleration: LinearVelocity) {
        pathingConfig = KTrajectoryConfig(velocity, maxAcceleration).apply { addConstraints(pathingConfig.constraints) }
        KTrajectory.generalConfig = pathingConfig
    }

    fun applyKinematics(kinematics: DifferentialDriveKinematics) {
        pathingConfig.setKinematics(kinematics)
    }

    fun applyKinematics(kinematics: MecanumDriveKinematics) {
        pathingConfig.setKinematics(kinematics)
    }

    fun applyKinematics(kinematics: SwerveDriveKinematics) {
        pathingConfig.setKinematics(kinematics)
    }

    // ----- public variables ----- //
    // location
    val heading: Angle  // what direction the robot is facing
        inline get() = pose.rotation.k
    var pose: Pose2d  // the location and direction of the robot
        get() = if (useOdometry) odometry.poseMeters else poseEstimator.estimatedPosition
        set(value) {
            if (useOdometry) odometry.resetPosition(value, gyro.heading.w)
            else poseEstimator.resetPosition(value, gyro.heading.w)

            if (Game.sim) KField2d.robotPose = value
        }
    val position: Translation2d  // the estimated location of the robot
        inline get() = pose.translation
    val odometry = DifferentialDriveOdometry(gyro.heading.w).apply {
        resetPosition(startPose, gyro.heading.w)
    }

    /**
     * Update position based on estimated motion
     */
    fun update(speeds: DifferentialDriveWheelSpeeds, leftPosition: Length, rightPosition: Length) {  // estimate motion
        if (useOdometry) odometry.update(gyro.heading.w, leftPosition.meters, rightPosition.meters)
        else poseEstimator.update(gyro.heading.w, speeds, leftPosition.meters, rightPosition.meters)
    }


    /**
     * Update position based on estimated motion
     */
    fun update(leftPosition: Length, rightPosition: Length) {  // estimate motion
        if (useOdometry) odometry.update(gyro.heading.w, leftPosition.meters, rightPosition.meters)
        else poseEstimator.update(heading.w, Drivetrain.wheelSpeeds, leftPosition.meters, rightPosition.meters)
    }

    /**
     * Update position based on a different position guess
     *
     * @param globalPosition the detected pose of the Robot
     * @param time the time of the detection
     */
    fun update(globalPosition: Pose2d, time: Time) {  // apply global position update
        when (trackingMode) {
            TrackingMode.Fancy -> poseEstimator.addVisionMeasurement(globalPosition, time.milliseconds)
            TrackingMode.DumbBoth -> odometry.resetPosition(globalPosition, heading.w)
            else -> log("Global position updates not configured", LogMode.WARN)
        }
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "pose" to pose
        )
    }
}
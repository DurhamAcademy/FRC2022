package frc.kyberlib.auto

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N5
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.kyberlib.auto.trajectory.KTrajectory
import frc.kyberlib.auto.trajectory.KTrajectoryConfig
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.command.LogMode
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.milli
import frc.kyberlib.math.units.string
import frc.kyberlib.math.units.zeroPose
import frc.kyberlib.sensors.gyros.KGyro
import frc.kyberlib.simulation.field.KField2d
import frc.robot.Constants

enum class TrackingMode {
    Fast, Fancy, DumbBoth
}
/**
 * Class to store and update robot navigation information
 */
class Navigator(private val gyro: KGyro, startPose: Pose2d = zeroPose, private val trackingMode: TrackingMode) : Debug {
    companion object { var instance: Navigator? = null }
    private val useOdometry = trackingMode != TrackingMode.Fancy
    init {
        instance = this
        gyro.heading = startPose.rotation.k
    }

    /**
     * A probability calculator to guess where the robot is from odometer and vision updates
     */
    private val poseEstimator = DifferentialDrivePoseEstimator(gyro.heading.w, startPose,
        MatBuilder(N5.instance, N1.instance).fill(0.02, 0.02, 0.01, 0.02, 0.02),
        MatBuilder(N3.instance, N1.instance).fill(0.02, 0.01, 0.01),
        MatBuilder(N3.instance, N1.instance).fill(0.1, 0.1, 0.01))

    /**
     * A object with restrictions on how the robot will move
     */
    private var pathingConfig = KTrajectoryConfig(1000.metersPerSecond, 1000.metersPerSecond).apply { KTrajectory.generalConfig = this }

    fun applyMovementRestrictions(velocity: LinearVelocity, maxAcceleration: LinearVelocity) {
        pathingConfig = KTrajectoryConfig(velocity, maxAcceleration).apply { addConstraints(pathingConfig.constraints) }
        KTrajectory.generalConfig = pathingConfig
    }
    fun applyKinematics(kinematics: DifferentialDriveKinematics) { pathingConfig.setKinematics(kinematics) }
    fun applyKinematics(kinematics: MecanumDriveKinematics) { pathingConfig.setKinematics(kinematics) }
    fun applyKinematics(kinematics: SwerveDriveKinematics) { pathingConfig.setKinematics(kinematics) }

    // ----- public variables ----- //
    // location
    var heading: Angle  // what direction the robot is facing
        get() = gyro.heading
        set(value) {gyro.heading = value}
    var pose: Pose2d  // the location and direction of the robot
        get() = if(!useOdometry) poseEstimator.estimatedPosition else odometry.poseMeters
        set(value) {
            if(!useOdometry) poseEstimator.resetPosition(value, heading.w)
            else odometry.resetPosition(value, heading.w)
            KField2d.robotPose = value
        }
    val position: Translation2d  // the estimated location of the robot
        get() = pose.translation
    private val odometry = DifferentialDriveOdometry(Constants.START_POSE.rotation).apply {
        resetPosition(startPose, heading.w)
    }

    /**
     * Update position based on estimated motion
     */
    fun update(speeds: DifferentialDriveWheelSpeeds, leftPosition: Length, rightPosition: Length) {  // estimate motion
        if (!useOdometry) poseEstimator.update(heading.w, speeds, leftPosition.meters, rightPosition.meters)
        else odometry.update(gyro.heading.w, leftPosition.meters, rightPosition.meters)
    }
    /**
     * Update position based on a different position guess
     *
     * @param globalPosition the detected pose of the Robot
     * @param time the time of the detection
     */
    fun update(globalPosition: Pose2d, time: Time) {  // apply global position update
        SmartDashboard.putString("global pose", globalPosition.string)
        when(trackingMode) {
            TrackingMode.Fancy -> poseEstimator.addVisionMeasurement(globalPosition, time.milliseconds)
            TrackingMode.DumbBoth -> if(Game.OPERATED) odometry.resetPosition(globalPosition, heading.w)
            else -> log("Global position updates not configured", LogMode.WARN)
        }
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "pose" to pose
        )
    }
}
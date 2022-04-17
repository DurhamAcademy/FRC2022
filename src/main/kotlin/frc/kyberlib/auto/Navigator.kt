package frc.kyberlib.auto

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N5
import frc.kyberlib.TRACK_WIDTH
import frc.kyberlib.auto.trajectory.KTrajectory
import frc.kyberlib.auto.trajectory.KTrajectoryConfig
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.command.KRobot
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.zeroPose
import frc.kyberlib.mechanisms.drivetrain.dynamics.KDifferentialDriveDynamic
import frc.kyberlib.sensors.gyros.KGyro
import frc.kyberlib.simulation.field.KField2d

/**
 * Class to store and update robot navigation information
 */
class Navigator(val gyro: KGyro, startPose: Pose2d = zeroPose) : Debug {
    //    override val priority: DebugFilter = DebugFilter.Max
    companion object {
        var instance: Navigator? = null
    }

    init {
        instance = this
//        gyro.heading = startPose.rotation.k
    }

    var differentialDrive = true

    /**
     * A probability calculator to guess where the robot is from odometer and vision updates
     */
    private val difPoseEstimator = DifferentialDrivePoseEstimator(gyro.heading.w, startPose,
        MatBuilder(N5.instance, N1.instance).fill(0.02, 0.02, 0.01, 0.02, 0.02),
        MatBuilder(N3.instance, N1.instance).fill(0.02, 0.01, 0.01),
        MatBuilder(N3.instance, N1.instance).fill(0.1, 0.1, 0.01)
    )

    private val holonomicPoseEstimator = DrivePoseEstimator(
        gyro.heading.w, startPose
    )

    /**
     * A object with restrictions on how the robot will move
     */
    private var pathingConfig = KTrajectoryConfig(5.metersPerSecond, 5.metersPerSecond).apply { KTrajectory.generalConfig = this }

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
        get() = if(differentialDrive) difPoseEstimator.estimatedPosition else holonomicPoseEstimator.estimatedPosition
        set(value) {
            if (!differentialDrive) holonomicPoseEstimator.resetPosition(value, gyro.heading.w)
            else difPoseEstimator.resetPosition(value, gyro.heading.w)

            if (Game.sim) KField2d.robotPose = value
        }
    val position: Translation2d  // the estimated location of the robot
        inline get() = pose.translation

    /**
     * Update position based on estimated motion
     */
    fun update(speeds: DifferentialDriveWheelSpeeds, leftPosition: Length, rightPosition: Length) {  // estimate motion
        difPoseEstimator.update(gyro.heading.w, speeds, leftPosition.meters, rightPosition.meters)
        simUpdate(KDifferentialDriveDynamic.toChassisSpeed(speeds, TRACK_WIDTH).omegaRadiansPerSecond.radiansPerSecond)
    }

    fun update(speeds: ChassisSpeeds) {
        holonomicPoseEstimator.update(gyro.heading.w, speeds)
        simUpdate(speeds.omegaRadiansPerSecond.radiansPerSecond)
    }


    /**
     * Update position based on a different position guess
     *
     * @param globalPosition the detected pose of the Robot
     * @param time the time of the detection
     */
    fun update(globalPosition: Pose2d, time: Time) {  // apply global position update
        if(differentialDrive) difPoseEstimator.addVisionMeasurement(globalPosition, time.milliseconds)
        else holonomicPoseEstimator.addVisionMeasurement(globalPosition, time.milliseconds)
        simUpdate()
    }

    private fun simUpdate(spin: AngularVelocity=0.radiansPerSecond) {
        gyro.heading += spin * KRobot.period.seconds
        if(Game.sim) {
            KField2d.robotPose = pose
        }
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "pose" to pose
        )
    }
}
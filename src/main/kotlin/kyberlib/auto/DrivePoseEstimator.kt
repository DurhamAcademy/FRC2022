package kyberlib.auto

import edu.wpi.first.wpilibj.estimator.MecanumDrivePoseEstimator
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds
import edu.wpi.first.wpilibj.math.Discretization
import edu.wpi.first.wpilibj.math.StateSpaceUtil
import edu.wpi.first.wpiutil.WPIUtilJNI
import edu.wpi.first.wpiutil.math.MatBuilder
import edu.wpi.first.wpiutil.math.Matrix
import edu.wpi.first.wpiutil.math.Nat
import edu.wpi.first.wpiutil.math.VecBuilder
import edu.wpi.first.wpiutil.math.numbers.N1
import edu.wpi.first.wpiutil.math.numbers.N3
import kyberlib.math.units.zeroPose

class DrivePoseEstimator(
    gyroAngle: Rotation2d,
    initialPoseMeters: Pose2d = zeroPose,
    stateStdDevs: Matrix<N3, N1> = MatBuilder(N3.instance, N1.instance).fill(0.02, 0.02, 0.01),
    localMeasurementStdDevs: Matrix<N1, N1> = MatBuilder(N1.instance, N1.instance).fill(0.02),
    visionMeasurementStdDevs: Matrix<N3, N1> = MatBuilder(N3.instance, N1.instance).fill(0.1, 0.1, 0.01)
) {
    // me scuffing drive Pose
    val cheesy = MecanumDriveKinematics(
        Translation2d(-1.0, 1.0),
        Translation2d(1.0, 1.0),
        Translation2d(-1.0, -1.0),
        Translation2d(1.0, -1.0)
    )
    private val internal = MecanumDrivePoseEstimator(gyroAngle, initialPoseMeters, cheesy, stateStdDevs, localMeasurementStdDevs, visionMeasurementStdDevs)


    /**
     * Sets the pose estimator's trust of global measurements. This might be used to change trust in
     * vision measurements after the autonomous period, or to change trust as distance to a vision
     * target increases.
     *
     * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
     * numbers to trust global measurements from vision less. This matrix is in the form [x, y,
     * theta]^T, with units in meters and radians.
     */
    fun setVisionMeasurementStdDevs(visionMeasurementStdDevs: Matrix<N3, N1>) {
        internal.setVisionMeasurementStdDevs(visionMeasurementStdDevs)
    }

    /**
     * Resets the robot's position on the field.
     *
     *
     * You NEED to reset your encoders (to zero) when calling this method.
     *
     *
     * The gyroscope angle does not need to be reset in the user's robot code. The library
     * automatically takes care of offsetting the gyro angle.
     *
     * @param poseMeters The position on the field that your robot is at.
     * @param gyroAngle The angle reported by the gyroscope.
     */
    fun resetPosition(poseMeters: Pose2d, gyroAngle: Rotation2d) {
        internal.resetPosition(poseMeters, gyroAngle)
    }

    /**
     * Gets the pose of the robot at the current time as estimated by the Unscented Kalman Filter.
     *
     * @return The estimated robot pose in meters.
     */
    fun getEstimatedPosition(): Pose2d {
        return internal.estimatedPosition
    }

    /**
     * Add a vision measurement to the Unscented Kalman Filter. This will correct the odometry pose
     * estimate while still accounting for measurement noise.
     *
     *
     * This method can be called as infrequently as you want, as long as you are calling [ ][MecanumDrivePoseEstimator.update] every loop.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
     * don't use your own time source by calling [MecanumDrivePoseEstimator.updateWithTime]
     * then you must use a timestamp with an epoch since FPGA startup (i.e. the epoch of this
     * timestamp is the same epoch as Timer.getFPGATimestamp.) This means that you should use
     * Timer.getFPGATimestamp as your time source or sync the epochs.
     */
    fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double) {
        internal.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds)
    }

    /**
     * Add a vision measurement to the Unscented Kalman Filter. This will correct the odometry pose
     * estimate while still accounting for measurement noise.
     *
     *
     * This method can be called as infrequently as you want, as long as you are calling [ ][MecanumDrivePoseEstimator.update] every loop.
     *
     *
     * Note that the vision measurement standard deviations passed into this method will continue
     * to apply to future measurements until a subsequent call to [ ][MecanumDrivePoseEstimator.setVisionMeasurementStdDevs] or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
     * don't use your own time source by calling [MecanumDrivePoseEstimator.updateWithTime]
     * then you must use a timestamp with an epoch since FPGA startup (i.e. the epoch of this
     * timestamp is the same epoch as Timer.getFPGATimestamp.) This means that you should use
     * Timer.getFPGATimestamp as your time source in this case.
     * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
     * numbers to trust global measurements from vision less. This matrix is in the form [x, y,
     * theta]^T, with units in meters and radians.
     */
    fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double, visionMeasurementStdDevs: Matrix<N3, N1>) {
        internal.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs)
    }

    /**
     * Updates the the Unscented Kalman Filter using only wheel encoder information. This should be
     * called every loop, and the correct loop period must be passed into the constructor of this
     * class.
     *
     * @param gyroAngle The current gyro angle.
     * @param chassisSpeeds The current speeds of the drivetrain.
     * @return The estimated pose of the robot in meters.
     */
    fun update(gyroAngle: Rotation2d, chassisSpeeds: ChassisSpeeds): Pose2d {
        return internal.update(gyroAngle, cheesy.toWheelSpeeds(chassisSpeeds))
    }

    /**
     * Updates the the Unscented Kalman Filter using only wheel encoder information. This should be
     * called every loop, and the correct loop period must be passed into the constructor of this
     * class.
     *
     * @param currentTimeSeconds Time at which this method was called, in seconds.
     * @param gyroAngle The current gyroscope angle.
     * @param chassisSpeeds The current speeds of the drivetrain.
     * @return The estimated pose of the robot in meters.
     */
    fun updateWithTime(currentTimeSeconds: Double, gyroAngle: Rotation2d, chassisSpeeds: ChassisSpeeds): Pose2d {
        return internal.updateWithTime(currentTimeSeconds, gyroAngle, cheesy.toWheelSpeeds(chassisSpeeds))
    }
}
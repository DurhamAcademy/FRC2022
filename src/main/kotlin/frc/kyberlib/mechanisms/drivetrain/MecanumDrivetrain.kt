package frc.kyberlib.mechanisms.drivetrain

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.MecanumDriveKinematics
import edu.wpi.first.math.kinematics.MecanumDriveOdometry
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.auto.Navigator
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.debugValues
import frc.kyberlib.math.units.extensions.metersPerSecond
import frc.kyberlib.math.units.extensions.w
import frc.kyberlib.motorcontrol.KMotorController

/**
 * Pre-made Drivetrain for MecanumWheels.
 * @param packages pairs of <wheelPosition, wheel motor> (currently requires 4 motors in frontLeft, frontRight, backLeft, backRight order)
 * @param gyro a KGyro that will provide heading information
 */
abstract class MecanumDrivetrain : KDrivetrain, SubsystemBase(), Debug {
    abstract val frontLeft: KMotorController
    abstract val frontRight: KMotorController
    abstract val backLeft: KMotorController
    abstract val backRight: KMotorController
    abstract val locations: Array<Translation2d>

    // changing this will normalize the top speed that a motor can have
    var maxVelocity = 0.metersPerSecond

    // controls
    private val kinematics = MecanumDriveKinematics(locations[0], locations[1], locations[2], locations[3])
    private val odometry = MecanumDriveOdometry(kinematics, Navigator.instance!!.heading.w)

    // useful info
    private val mecanumDriveWheelSpeeds
        get() = MecanumDriveWheelSpeeds(frontLeft.linearVelocity.metersPerSecond, frontRight.linearVelocity.metersPerSecond, backLeft.linearVelocity.metersPerSecond, backRight.linearVelocity.metersPerSecond)
    var pose: Pose2d
        set(value) { odometry.resetPosition(value, Navigator.instance!!.heading.w) }
        get() = odometry.poseMeters
    var heading
        get() = Navigator.instance!!.heading
        set(value) {Navigator.instance!!.heading = value}
    override val chassisSpeeds: ChassisSpeeds
        get() = kinematics.toChassisSpeeds(mecanumDriveWheelSpeeds)

    // drive functions
    override fun drive(speeds: ChassisSpeeds) {
        drive(speeds, false)
    }

    fun drive(speeds: ChassisSpeeds, fieldRelative: Boolean = false) {
        if (fieldRelative) {
            // field relative means x, y of the field instead of forward, side from the robot
            val adjusted = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, Navigator.instance!!.heading.w)
            drive(kinematics.toWheelSpeeds(adjusted))
        }
        else drive(kinematics.toWheelSpeeds(speeds))
    }

    fun drive(speeds: MecanumDriveWheelSpeeds) {
        if (maxVelocity.value > 0)
            speeds.desaturate(maxVelocity.metersPerSecond)
        frontLeft.linearVelocity = speeds.frontLeftMetersPerSecond.metersPerSecond
        frontRight.linearVelocity = speeds.frontRightMetersPerSecond.metersPerSecond
        backLeft.linearVelocity = speeds.rearLeftMetersPerSecond.metersPerSecond
        backRight.linearVelocity = speeds.rearRightMetersPerSecond.metersPerSecond
    }

    override fun periodic() {
        odometry.update(Navigator.instance!!.heading.w, mecanumDriveWheelSpeeds)
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "pose" to pose.debugValues,
            "speed" to chassisSpeeds,
            "Front Left" to frontLeft,
            "Front Right" to frontRight,
            "Back Left" to backRight,
            "Back Right" to backLeft,
        )
    }

}
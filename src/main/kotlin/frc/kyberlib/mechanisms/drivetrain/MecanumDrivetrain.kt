package frc.kyberlib.mechanisms.drivetrain

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.metersPerSecond
import frc.kyberlib.mechanisms.drivetrain.dynamics.KMecanumDynamics
import frc.kyberlib.motorcontrol.KMotorController

/**
 * Pre-made Drivetrain for MecanumWheels.
 * @param packages pairs of <wheelPosition, wheel motor> (currently requires 4 motors in frontLeft, frontRight, backLeft, backRight order)
 * @param gyro a KGyro that will provide heading information
 */
abstract class MecanumDrivetrain : KDrivetrain(), Debug {
    abstract val frontLeft: KMotorController
    abstract val frontRight: KMotorController
    abstract val backLeft: KMotorController
    abstract val backRight: KMotorController
    abstract val locations: Array<Translation2d>

    override val dynamics: KMecanumDynamics = KMecanumDynamics(frontLeft, frontRight, backLeft, backRight, locations)

    // changing this will normalize the top speed that a motor can have
    var maxVelocity = 0.metersPerSecond

    // useful info
    private val mecanumDriveWheelSpeeds
        inline get() = dynamics.wheelSpeeds

    fun drive(speeds: ChassisSpeeds, fieldRelative: Boolean = false) {
        dynamics.drive(speeds, fieldRelative)
    }

    fun drive(speeds: MecanumDriveWheelSpeeds) {
        if (maxVelocity.value > 0)
            speeds.desaturate(maxVelocity.metersPerSecond)
        dynamics.drive(speeds)
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "speed" to chassisSpeeds,
            "Front Left" to frontLeft,
            "Front Right" to frontRight,
            "Back Left" to backRight,
            "Back Right" to backLeft,
        )
    }

}
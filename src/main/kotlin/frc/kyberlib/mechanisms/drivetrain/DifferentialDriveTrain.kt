package frc.kyberlib.mechanisms.drivetrain

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import frc.kyberlib.TRACK_WIDTH
import frc.kyberlib.auto.Navigator
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.debugValues
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.mechanisms.drivetrain.dynamics.KDifferentialDriveDynamic
import frc.kyberlib.motorcontrol.KMotorController

/**
 * Pre-made DifferentialDrive Robot.
 * @param leftMotors array of all the motors on the left side of the robot. They will all follow the first
 * @param rightMotors array of all the motors on the right side of the robot. They will all follow the first
 * @param configs information about the physical desciption of this drivetrain
 * @param gyro KGyro to provide heading information
 */
abstract class DifferentialDriveTrain : KDrivetrain(), Debug {

    abstract override val dynamics: KDifferentialDriveDynamic

    val wheelSpeeds
        inline get() = dynamics.wheelSpeeds

    fun drive(speeds: DifferentialDriveWheelSpeeds) {
        dynamics.drive(speeds)
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "pose" to Navigator.instance!!.pose.debugValues,
            "speed" to chassisSpeeds.debugValues,
        )
    }
}

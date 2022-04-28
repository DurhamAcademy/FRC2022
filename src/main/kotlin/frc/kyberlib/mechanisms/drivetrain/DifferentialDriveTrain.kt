package frc.kyberlib.mechanisms.drivetrain

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import frc.kyberlib.command.Debug
import frc.kyberlib.mechanisms.drivetrain.dynamics.KDifferentialDriveDynamic

/**
 * Pre-made DifferentialDrive Robot.
*/
abstract class DifferentialDriveTrain : KDrivetrain(), Debug {
    abstract override val dynamics: KDifferentialDriveDynamic

    inline val wheelSpeeds
        get() = dynamics.wheelSpeeds

    fun drive(speeds: DifferentialDriveWheelSpeeds) {
        dynamics.drive(speeds)
    }
}

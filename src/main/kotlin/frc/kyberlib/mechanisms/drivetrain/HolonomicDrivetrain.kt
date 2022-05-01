package frc.kyberlib.mechanisms.drivetrain

import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.LinearVelocity
import frc.kyberlib.mechanisms.drivetrain.dynamics.KHolonomicDriveDynamics

/**
 * Pre-made drivetrain for anything holonomic (can move sideways)
 */
abstract class HolonomicDrivetrain : KDrivetrain(), Debug {
    abstract override val dynamics: KHolonomicDriveDynamics

    override fun drive(chassisSpeeds: ChassisSpeeds) {
        dynamics.drive(chassisSpeeds, false)
    }

    fun drive(speeds: ChassisSpeeds, fieldRelative: Boolean) {
        dynamics.drive(speeds, fieldRelative)
    }
}
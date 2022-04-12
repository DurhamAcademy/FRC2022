package frc.kyberlib.mechanisms.drivetrain

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.auto.Navigator
import frc.kyberlib.auto.pathing.Pathfinder
import frc.kyberlib.auto.trajectory.KTrajectory
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.metersPerSecond
import frc.kyberlib.math.units.extensions.radiansPerSecond
import frc.kyberlib.math.units.string
import frc.kyberlib.math.units.towards
import frc.kyberlib.mechanisms.drivetrain.dynamics.KDriveDynamics

/**
 * Interface for all pre-made Drivetrains
 */
abstract class KDrivetrain : SubsystemBase(), Debug {
    abstract val dynamics: KDriveDynamics

    fun drive(chassisSpeeds: ChassisSpeeds) {
        dynamics.drive(chassisSpeeds)
    }

    fun drive(trajectory: KTrajectory) = dynamics.drive(trajectory)

    /**
     * Drive to a pose
     * @param goal the pose to drive to
     * @param direct whether to drive straight to goal or do pathplanning for obstacle avoidance
     */
    fun driveTo(goal: Pose2d, direct: Boolean = true) = dynamics.driveTo(goal, direct)
    /**
     * Drive to a pose
     * @param goal the position to drive to
     * @param direct whether to drive straight to goal or do pathplanning for obstacle avoidance
     */
    fun driveTo(goal: Translation2d, direct: Boolean = true) = dynamics.driveTo(goal, direct)

    val chassisSpeeds: ChassisSpeeds
        inline get() = dynamics.chassisSpeeds

    override fun periodic() {
        dynamics.updateNavigation()
    }

}
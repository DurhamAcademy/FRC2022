package frc.kyberlib.mechanisms.drivetrain.dynamics

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.kyberlib.auto.Navigator
import frc.kyberlib.auto.pathing.Pathfinder
import frc.kyberlib.auto.trajectory.KTrajectory
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.AngularVelocity
import frc.kyberlib.math.units.extensions.LinearVelocity
import frc.kyberlib.math.units.extensions.metersPerSecond
import frc.kyberlib.math.units.extensions.radiansPerSecond
import frc.kyberlib.math.units.string
import frc.kyberlib.math.units.towards

abstract class KDriveDynamics : Debug {
    companion object {
        var instance: KDriveDynamics? = null
    }
    abstract fun drive(chassisSpeeds: ChassisSpeeds)
    abstract fun drive(path: KTrajectory)
    abstract fun stop()

    abstract val chassisSpeeds: ChassisSpeeds

    fun drive(fwd: LinearVelocity, turn: AngularVelocity) {
        drive(ChassisSpeeds(fwd.metersPerSecond, 0.0, turn.radiansPerSecond))
    }

    open fun updateNavigation() {
        Navigator.instance!!.update(this)
    }

    var activeTrajectory: KTrajectory? = null
    /**
     * Drive to a pose
     * @param goal the pose to drive to
     * @param direct whether to drive straight to goal or do pathplanning for obstacle avoidance
     */
    fun driveTo(goal: Pose2d, direct: Boolean = true) {
        if(activeTrajectory != null && activeTrajectory!!.states.last().poseMeters != goal) {
            val currentLocation = Navigator.instance!!.pose
            if (direct)
                KTrajectory(goal.string, currentLocation, emptyList(), goal)
            else {
                activeTrajectory = Pathfinder.pathTo(currentLocation, goal)
            }
        }
        activeTrajectory
        drive(activeTrajectory!!)
    }
    /**
     * Drive to a pose
     * @param goal the position to drive to
     * @param direct whether to drive straight to goal or do pathplanning for obstacle avoidance
     */
    fun driveTo(goal: Translation2d, direct: Boolean = true) {
        if(activeTrajectory != null && activeTrajectory!!.states.last().poseMeters.translation != goal) {
            val currentLocation = Navigator.instance!!.pose
            if (direct)
                KTrajectory(goal.toString(), currentLocation, emptyList(), Pose2d(goal, currentLocation.translation.towards(goal)))
            else {
                activeTrajectory = Pathfinder.pathTo(currentLocation, goal)
            }
        }
        activeTrajectory
        drive(activeTrajectory!!)
    }
    /**
     * remove the active trajectory data
     */
    fun clearTrajectory() {activeTrajectory = null}
}
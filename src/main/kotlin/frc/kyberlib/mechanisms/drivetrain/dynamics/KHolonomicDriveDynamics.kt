package frc.kyberlib.mechanisms.drivetrain.dynamics

import edu.wpi.first.math.controller.HolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.kyberlib.auto.Navigator
import frc.kyberlib.auto.trajectory.KTrajectory
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.*

abstract class KHolonomicDriveDynamics(var fieldRelativeOffset: Angle) : KDriveDynamics() {
    init {
        Navigator.instance!!.differentialDrive = false
    }

    val xPID = PIDController(1.0, 0.0, 0.0)
    val yPID = PIDController(1.0, 0.0, 0.0)
    val thetaPID = ProfiledPIDController(1.0, 0.0, 0.0,
        TrapezoidProfile.Constraints(
            1.rotationsPerSecond.radiansPerSecond,  // vel
            1.rotationsPerSecond.radiansPerSecond  // acc
        )
    )
    private var autoStartTime = Game.time
    private val controller = HolonomicDriveController(xPID, yPID, thetaPID)
    private var trajectory: KTrajectory? = null
    /**
     * generates a command to follow a trajectory
     */
    override fun drive(path: KTrajectory) {
        if(trajectory == null || path != trajectory) {
            trajectory = path
            autoStartTime = Game.time
        }
        val curTime = Game.time - autoStartTime
        val desiredState: Trajectory.State = trajectory!!.sample(curTime.seconds)

        val targetChassisSpeeds: ChassisSpeeds = controller.calculate(
            Navigator.instance!!.pose,
            desiredState, trajectory!!.states.last().poseMeters.rotation)  // idk about this
        drive(targetChassisSpeeds, false)
    }

    override fun updateNavigation() {
        Navigator.instance!!.update(chassisSpeeds)
    }

    override fun drive(chassisSpeeds: ChassisSpeeds) {
        drive(chassisSpeeds, true)
    }

    fun drive(speeds: ChassisSpeeds, fieldRelative: Boolean=true) {
        if(fieldRelative) driveFieldRelative(speeds)
        else driveRobotRelative(speeds)
    }

    abstract fun driveRobotRelative(speeds: ChassisSpeeds)

    fun driveFieldRelative(speeds: ChassisSpeeds) {
        val robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            (Navigator.instance!!.heading - fieldRelativeOffset).w
        )
        driveRobotRelative(robotSpeeds)
    }
}
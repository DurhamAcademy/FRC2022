package frc.team6502.robot.commands.drive

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.subsystems.Drivetrain
import kyberlib.auto.Navigator
import kyberlib.auto.pathing.Pathfinder
import kyberlib.auto.trajectory.KTrajectory
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.string
import kyberlib.simulation.field.KField2d


/**
 * Automatically path and drive to a pose when called
 * @param targetPose the pose to drive to
 */
class AutoDrive(var targetPose: Pose2d) : CommandBase() {
    companion object {
        private var calculator = RamseteController(0.0, 0.0)  // these are the recommended values

        fun updateRamsete(beta: Double, zeta: Double) {
            calculator = RamseteController(beta, zeta)
        }
    }
    val simple = true
    constructor(position: Translation2d) : this(Pose2d(position, 0.degrees)) {
        rotationInvariant = true
    }

    constructor(trajectory: Trajectory) : this(trajectory.states.last().poseMeters) {
        this.trajectory = trajectory
    }

    init {
        assert(KField2d.inField(targetPose.translation)) {"Invalid location selected. The robot tried to drive to ${targetPose.translation}, which is outside of the field"}
        addRequirements(Drivetrain)
    }
    private var rotationInvariant = false

    lateinit var trajectory: Trajectory
    private val timer = Timer()

    override fun initialize() {
        if (!this::trajectory.isInitialized)
            trajectory = if (simple) KTrajectory("simpleTraj", listOf(Navigator.instance!!.pose, targetPose))
            else if (rotationInvariant) Pathfinder.pathTo(Navigator.instance!!.pose, targetPose.translation)
            else Pathfinder.pathTo(Navigator.instance!!.pose, targetPose)
        KField2d.trajectory = trajectory
        timer.start()
    }

    override fun execute() {
        val targetSpeed = calculator.calculate(Navigator.instance!!.pose, trajectory.sample(timer.get()))
        println("pose: ${Navigator.instance!!.pose.string}, expected: ${trajectory.sample(timer.get()).poseMeters.string}")
        Drivetrain.drive(targetSpeed)
//        trajectory = PathPlanner.updateTrajectory(trajectory) - this should be necesary until moving obstabcles
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(trajectory.totalTimeSeconds)
    }
}
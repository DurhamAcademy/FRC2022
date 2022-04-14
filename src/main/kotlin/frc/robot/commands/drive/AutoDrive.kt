package frc.robot.commands.drive

import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.auto.Navigator
import frc.kyberlib.auto.trajectory.KTrajectory
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.rotationsPerSecond
import frc.kyberlib.math.units.extensions.seconds
import frc.kyberlib.simulation.field.KField2d
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Intaker
import frc.robot.subsystems.Shooter


/**
 * Automatically path and drive to a pose when called
 * @param targetPose the pose to drive to
 */
class AutoDrive(var targetPose: Pose2d, private val intaking: Boolean = true) : CommandBase() {
    private var calculator = RamseteController(2.0, 0.7)  // these are the recommended values
    val simple = true

    constructor(trajectory: KTrajectory) : this(trajectory.states.last().poseMeters) {
        this.trajectory = trajectory
    }

    init {
        assert(KField2d.inField(targetPose.translation)) { "Invalid location selected. The robot tried to drive to ${targetPose.translation}, which is outside of the field" }
        addRequirements(Drivetrain, Intaker)
    }

    private var rotationInvariant = false

    lateinit var trajectory: KTrajectory
    private val timer = Timer()

    /**
     * Prepares the trajectory that the robot will follow
     */
    override fun initialize() {
        if (!this::trajectory.isInitialized)
//            trajectory = if (simple) KTrajectory("simpleTraj", listOf(Navigator.instance!!.pose, targetPose))
//            else if (rotationInvariant) Pathfinder.pathTo(Navigator.instance!!.pose, targetPose.translation)
//            else Pathfinder.pathTo(Navigator.instance!!.pose, targetPose)
            KField2d.trajectory = trajectory
        if (intaking) {
            Intaker.intakeMotor.percent = Constants.INTAKE_PERCENT
            Intaker.deployed = true
        }
        timer.start()
        timer.reset()
    }

    /**
     * Follow the generated trajectory
     */
    override fun execute() {
        Drivetrain.drive(trajectory)
    }

    override fun end(interrupted: Boolean) {
        if (intaking) {
            Intaker.intakeMotor.stop()
            Intaker.deployed = false
        }
        Drivetrain.stop()
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(trajectory.totalTimeSeconds)
    }
}
package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.kyberlib.auto.trajectory.TrajectoryManager
import frc.kyberlib.command.Game
import frc.kyberlib.command.KRobot
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.math.units.string
import frc.kyberlib.math.units.zeroPose
import frc.kyberlib.simulation.field.KField2d
import frc.robot.commands.drive.AutoDrive
import frc.robot.commands.shooter.Shoot
import frc.robot.subsystems.*
import java.io.File

class Robot : KRobot() {
    private var autoCommand: Command? = null
    override fun robotInit() {
        RobotContainer  // initializes object

        Climber.leftWinch.resetPosition(0.degrees)
        Climber.rightWinch.resetPosition(0.degrees)
    }

    override fun robotPeriodic() {
        if (Game.real) {
            // log relevant stuff
            SmartDashboard.putNumber("gyro", RobotContainer.gyro.heading.degrees)
            SmartDashboard.putBoolean("visible", Limelight.targetVisible)
            SmartDashboard.putBoolean("shooter ready", Shooter.ready)
            SmartDashboard.putNumber("distance", Limelight.distance.meters)
            SmartDashboard.putString("pose", (RobotContainer.navigation.position - Constants.HUB_POSITION).string)
        }
    }

    override fun disabledInit() {
        Drivetrain.stop()
    }

    override fun autonomousInit() {
        RobotContainer.startTime = Game.time
        // prepare to pick up balls
        Intaker.deployed = true
        Conveyor.conveyor.percent = 0.05
        Intaker.intakeMotor.percent = Constants.INTAKE_PERCENT
        // get auto commadn
        val auto = loadRoutine(RobotContainer.autoChooser.selected)//RobotContainer.autoChooser.selected!!)
        auto.schedule()
        autoCommand = auto
    }

    private fun reset(pose: Pose2d) {
        // reset navigation to a specific pose
        RobotContainer.navigation.pose = pose
    }

    // todo: separate intake system
    private fun loadRoutine(routine: String): SequentialCommandGroup {
        val command = SequentialCommandGroup()
        TrajectoryManager.routines
        val f = File("${TrajectoryManager.AUTO_PATH}/$routine")
        f.readLines().forEach {
            when (it) {
                "Shot" -> command.addCommands(Shoot())
                else -> command.addCommands(AutoDrive(TrajectoryManager[it]!!))
            }
        }

        // resets our position assuming we are in the right spot
        var pose = zeroPose
        for (it in f.readLines()) {
            if (it != "Shot") pose = TrajectoryManager[it]!!.initialPose
            break
        }

        reset(pose)
        return command
    }

    override fun autonomousExit() {
        KField2d.trajectory = null
        autoCommand?.cancel()
        Intaker.deployed = false
        Intaker.intakeMotor.stop()
    }
}
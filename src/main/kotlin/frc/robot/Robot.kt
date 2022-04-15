package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.kyberlib.auto.AutoDrive
import frc.kyberlib.auto.trajectory.TrajectoryManager
import frc.kyberlib.command.Game
import frc.kyberlib.command.KRobot
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.math.units.zeroPose
import frc.kyberlib.simulation.field.KField2d
import frc.robot.commands.shooter.AutoShot
import frc.robot.subsystems.*
import java.io.File

class Robot : KRobot() {
    private var autoCommand: Command? = null
    override fun robotInit() {
        RobotContainer  // initializes object

        Climber.leftWinch.resetPosition(0.degrees)
        Climber.rightWinch.resetPosition(0.degrees)
    }

    override fun autonomousInit() {
        RobotContainer.startTime = Game.time
        // zero Turret
        Turret.zeroTurret()
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
        if (Game.real) {
            Drivetrain.leftMaster.resetPosition(0.meters)
            Drivetrain.rightMaster.resetPosition(0.meters)
        } else {
//            RobotContainer.gyro.heading = pose.rotation.k
        }
        RobotContainer.navigation.pose = pose
    }

    // todo: separate intake system
    private fun loadRoutine(routine: String): SequentialCommandGroup {
        val command = SequentialCommandGroup()
        val f = File("${TrajectoryManager.AUTO_PATH}/$routine")
        f.readLines().forEach {
            when (it) {
                "Shot" -> command.addCommands(AutoShot())
                else -> command.addCommands(AutoDrive(Drivetrain, TrajectoryManager[it]!!))
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
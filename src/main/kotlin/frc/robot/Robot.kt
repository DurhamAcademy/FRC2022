package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
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
import frc.robot.commands.shooter.AutoShot
import frc.robot.commands.shooter.Dispose
import frc.robot.commands.shooter.FullAutoFire
import frc.robot.commands.turret.ZeroTurret
import frc.robot.subsystems.*
import java.io.File

class Robot : KRobot() {
    private var autoCommand: Command? = null
    override fun robotInit() {
        RobotContainer

        Climber.leftWinch.resetPosition(0.degrees)
        Climber.rightWinch.resetPosition(0.degrees)
    }

    override fun robotPeriodic() {
        RobotContainer.leds.update()
    }

    override fun disabledInit() {
        Drivetrain.stop()
    }

    override fun enabledInit() {
//        ZeroTurret.schedule(false)

    }

    override fun teleopInit() {
        // TODO: probably want to get rid of this at the event, but it should
        // handle being on fms just fine. probably just want to get rid of it
        // just in case though
        if (!DriverStation.isFMSAttached()) reset(Constants.START_POSE)
    }

    override fun teleopPeriodic() {
//        SmartDashboard.putBoolean("hall", RobotContainer.turretLimit.get())
        if (Game.real) {
            SmartDashboard.putNumber("gyro", RobotContainer.gyro.heading.degrees)
            SmartDashboard.putBoolean("visible", Turret.targetVisible)
            SmartDashboard.putBoolean("shooter ready", Shooter.ready)
            SmartDashboard.putBoolean("turret ready", Turret.ready)
            Turret.targetDistance?.meters?.let { SmartDashboard.putNumber("distance", it) }
            SmartDashboard.putString("pose", (RobotContainer.navigation.position - Constants.HUB_POSITION).string)
        }
    }

    override fun autonomousInit() {
        Turret.isZeroed = false
        ZeroTurret.schedule(false)
        Intaker.deployed = true
        Conveyor.conveyor.percent = 0.05
        Intaker.intakeMotor.percent = Constants.INTAKE_PERCENT
        val auto = loadRoutine(RobotContainer.autoChooser.selected)//RobotContainer.autoChooser.selected!!)
        auto.schedule()
        autoCommand = auto
    }

    private fun reset(pose: Pose2d) {
        if (Game.real) {
            Drivetrain.leftMaster.resetPosition(0.meters)
            Drivetrain.rightMaster.resetPosition(0.meters)
        } else {
//            RobotContainer.gyro.heading = pose.rotation.k
        }
        RobotContainer.navigation.pose = pose
    }

    private fun loadRoutine(routine: String): SequentialCommandGroup {
        val command = SequentialCommandGroup()
        val f = File("${TrajectoryManager.AUTO_PATH}/$routine")
        f.readLines().forEach {
            when (it) {
                "Shot" -> command.addCommands(AutoShot())
                "Dispose" -> command.addCommands(Dispose)
                else -> command.addCommands(AutoDrive(TrajectoryManager[it]!!))
            }
        }
        if (!f.readLines().contains("Shot")) FullAutoFire().schedule()

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
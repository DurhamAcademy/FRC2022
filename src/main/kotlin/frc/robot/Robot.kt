package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.kyberlib.auto.trajectory.TrajectoryManager
import frc.kyberlib.command.Game
import frc.kyberlib.command.KRobot
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.k
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.math.units.string
import frc.kyberlib.math.units.zeroPose
import frc.kyberlib.simulation.field.KField2d
import frc.robot.commands.drive.AutoDrive
import frc.robot.commands.shooter.AutoShot
import frc.robot.commands.shooter.FullAutoFire
import frc.robot.commands.turret.ZeroTurret
import frc.robot.subsystems.*
import java.io.File

class Robot : KRobot() {
    private var autoCommand : Command? = null
    override fun robotInit() {
        RobotContainer

        Climber.leftWinch.resetPosition(0.degrees)
        Climber.rightWinch.resetPosition(0.degrees)
    }

    override fun disabledInit() {
        Drivetrain.stop()
    }

    override fun enabledInit() {
        ZeroTurret.schedule(false) // don't uncomment this until the Hall sensor is added or bad things might happen
        reset(Constants.START_POSE)
    }

    override fun teleopInit() {
    }

    private fun reset(pose: Pose2d) {
        Drivetrain.leftMaster.resetPosition(0.meters)
        Drivetrain.rightMaster.resetPosition(0.meters)
//        RobotContainer.gyro.heading = pose.rotation.k
        RobotContainer.navigation.pose = pose
    }

    override fun teleopPeriodic() {
//        SmartDashboard.putBoolean("hall", RobotContainer.turretLimit.get())
        if(Game.real) {
            SmartDashboard.putNumber("gyro", RobotContainer.gyro.heading.degrees)
            SmartDashboard.putBoolean("visible", Turret.targetVisible)
            SmartDashboard.putBoolean("shooter ready", Shooter.ready)
            SmartDashboard.putBoolean("turret ready", Turret.ready)
            Turret.targetDistance?.meters?.let { SmartDashboard.putNumber("distance", it) }
            SmartDashboard.putString("pose", RobotContainer.navigation.pose.string)
        }
    }

    override fun autonomousInit() {
        Intaker.deployed = true
        Intaker.intakeMotor.percent = Constants.INTAKE_PERCENT
        val auto = loadRoutine(RobotContainer.autoChooser.selected)//RobotContainer.autoChooser.selected!!)
        auto.schedule()
        autoCommand = auto
    }

    fun loadRoutine(routine: String): SequentialCommandGroup {
        val command = SequentialCommandGroup()
        val f = File("${TrajectoryManager.AUTO_PATH}/$routine")
        f.readLines().forEach {
            if(it == "Shot") command.addCommands(AutoShot())
            else command.addCommands(AutoDrive(TrajectoryManager[it]!!))
        }
        if (!f.readLines().contains("Shot")) FullAutoFire().schedule()

        // resets our position assuming we are in the right spot
        var pose = zeroPose
        for(it in f.readLines()) {
            if(it != "Shot") pose = TrajectoryManager[it]!!.initialPose
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
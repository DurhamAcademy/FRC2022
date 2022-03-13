package frc.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.kyberlib.auto.trajectory.TrajectoryManager
import frc.kyberlib.command.Game
import frc.kyberlib.command.KRobot
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.zeroPose
import frc.kyberlib.simulation.field.KField2d
import frc.robot.commands.drive.AutoDrive
import frc.robot.commands.shooter.AutoShot
import frc.robot.commands.shooter.FullAutoFire
import frc.robot.commands.turret.ZeroTurret
import frc.robot.subsystems.Climber
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Intaker
import frc.robot.subsystems.Turret
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
//        Turret.turret.resetPosition(0.degrees)  // remove this when you add zeroTurret back in
    }

    override fun teleopPeriodic() {
//        SmartDashboard.putBoolean("hall", RobotContainer.turretLimit.get())
        SmartDashboard.putNumber("gyro", RobotContainer.gyro.heading.degrees)
    }

    override fun autonomousInit() {
        Intaker.deployed = true
        Intaker.intakeMotor.percent = Constants.INTAKE_PERCENT
        // todo: make the chooser work
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
        if (Game.sim) {
            var pose = zeroPose
            for(it in f.readLines()) {
                if(it != "Shot") pose = TrajectoryManager[it]!!.initialPose
                break
            }
            RobotContainer.navigation.pose = pose

        }
        return command
    }

    override fun autonomousExit() {
        KField2d.trajectory = null
        autoCommand?.cancel()
    }
}
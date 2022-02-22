package frc.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.kyberlib.auto.trajectory.KTrajectory
import frc.kyberlib.command.Game
import frc.kyberlib.command.KRobot
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.simulation.Simulation
import frc.kyberlib.simulation.field.KField2d
import frc.robot.commands.drive.AutoDrive
import frc.robot.commands.turret.ZeroTurret
import frc.robot.subsystems.*

class Robot : KRobot() {
    private var autoCommand : Command? = null
    override fun robotInit() {
        RobotContainer

        Climber.leftWinch.resetPosition(0.degrees)
        Climber.rightWinch.resetPosition(0.degrees)

        if(Game.sim) {
            Simulation.instance.include(Drivetrain)
            Drivetrain.setupSim()
            Simulation.instance.include(Turret)
            Simulation.instance.include(Climber)
            Simulation.instance.include(Shooter)
        }
    }

    override fun disabledInit() {
        Drivetrain.stop()
    }

    override fun enabledInit() {
         ZeroTurret.schedule(false) // don't uncomment this until the Hall sensor is added or bad things might happen
    }

    override fun autonomousInit() {
        Intaker.deployed = true
        Intaker.intakeMotor.percent = Constants.INTAKE_PERCENT
        val traj = KTrajectory.load("4Ball")
        val auto = AutoDrive(traj)
        auto.schedule()
        autoCommand = auto
    }

    override fun autonomousExit() {
        KField2d.trajectory = null
        autoCommand?.cancel()
    }
}
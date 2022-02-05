package frc.team6502.robot

import frc.team6502.robot.commands.drive.AutoDrive
import frc.team6502.robot.commands.drive.Drive
import frc.team6502.robot.subsystems.Climber
import frc.team6502.robot.subsystems.Drivetrain
import frc.team6502.robot.subsystems.Shooter
import frc.team6502.robot.subsystems.Turret
import kyberlib.command.Game
import kyberlib.command.KRobot
import kyberlib.math.units.extensions.inches
import kyberlib.simulation.Simulation
import kyberlib.simulation.field.KField2d

class Robot : KRobot() {
    override fun robotInit() {
        RobotContainer

        Climber.leftWinch.resetPosition(0.inches)
        Climber.rightWinch.resetPosition(0.inches)

        if(Game.sim) {
            Simulation.instance.include(Drivetrain)
            Drivetrain.setupSim()
            Simulation.instance.include(Turret)
            Simulation.instance.include(Shooter.flywheelControl)
        }
    }

    override fun disabledInit() {
        Drivetrain.stop()
    }

    override fun teleopPeriodic() {
        RobotContainer.controller.debugDashboard()
//        Drive.execute()
    }

    override fun enabledInit() {
        // ZeroTurret.schedule(false) // don't uncomment this until the Hall sensor is added or bad things might happen
    }

    override fun teleopInit() {
        KField2d.trajectory = null
    }

    override fun autonomousInit() {
        AutoDrive(Constants.HUB_POSITION).schedule()
    }
}
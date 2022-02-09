package frc.robot

import frc.robot.commands.drive.AutoDrive
import frc.robot.commands.drive.Drive
import frc.robot.subsystems.Climber
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Turret
import frc.kyberlib.command.Game
import frc.kyberlib.command.KRobot
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.simulation.Simulation
import frc.kyberlib.simulation.field.KField2d

class Robot : KRobot() {
    override fun robotInit() {
        RobotContainer

        Climber.leftWinch.resetPosition(0.degrees)
        Climber.rightWinch.resetPosition(0.degrees)

        if(Game.sim) {
            Simulation.instance.include(Drivetrain)
            Drivetrain.setupSim()
            Simulation.instance.include(Turret)
            // Simulation.instance.include(Climber)
           Simulation.instance.include(Shooter)
        }
    }

    override fun disabledInit() {
        Drivetrain.stop()
    }

    override fun teleopPeriodic() {
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
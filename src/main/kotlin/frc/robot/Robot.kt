package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import frc.kyberlib.command.Game
import frc.kyberlib.command.KRobot
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.simulation.Simulation
import frc.kyberlib.simulation.field.KField2d
import frc.robot.commands.auto.Autoificate
import frc.robot.commands.drive.AutoDrive
import frc.robot.commands.turret.ZeroTurret
import frc.robot.subsystems.Climber
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Turret

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
        autoCommand?.cancel()
    }

    override fun teleopPeriodic() {
    }

    override fun enabledInit() {
        // ZeroTurret.schedule(false) // don't uncomment this until the Hall sensor is added or bad things might happen
    }

    override fun teleopInit() {
        KField2d.trajectory = null
        autoCommand?.cancel()
    }

    override fun autonomousInit() {
        ZeroTurret.schedule()
        Autoificate().schedule()
    }
}
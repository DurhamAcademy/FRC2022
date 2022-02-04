package frc.team6502.robot

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import frc.team6502.robot.commands.turret.ZeroTurret
import frc.team6502.robot.subsystems.Climber
import kyberlib.command.KRobot
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.inches

class Robot : KRobot() {
    override fun robotInit() {
        RobotContainer

        RobotContainer.navigation.pose = Pose2d(0.0, 0.0, 0.degrees)  // todo: update with starting pose

        Climber.leftWinch.resetPosition(0.inches)
        Climber.rightWinch.resetPosition(0.inches)
    }

    override fun disabledInit() {
        // set leds to red
    }

    override fun enabledInit() {
        // ZeroTurret.schedule(false) // don't uncomment this until the Hall sensor is added or bad things might happen
    }

    override fun autonomousInit() {
        val auto: Command = TODO()
        auto.schedule()
    }
}
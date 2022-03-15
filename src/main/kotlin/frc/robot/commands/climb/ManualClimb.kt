package frc.robot.commands.climb

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Climber

object ManualClimb : CommandBase() {
    init {
        addRequirements(Climber)
    }

    override fun initialize() {
        SmartDashboard.putNumber("left winch v", 0.0)
        SmartDashboard.putNumber("right winch v", 0.0)
        SmartDashboard.putNumber("left rot v", 0.0)
        SmartDashboard.putNumber("right rot v", 0.0)
        SmartDashboard.putBoolean("static", false)
    }

    override fun execute() {
        Climber.leftWinch.voltage = SmartDashboard.getNumber("left winch v", 0.0)
        Climber.rightWinch.voltage = SmartDashboard.getNumber("right winch v", 0.0)
        Climber.leftExtendable.voltage = SmartDashboard.getNumber("left rot v", 0.0)
        Climber.rightExtendable.voltage = SmartDashboard.getNumber("right rot v", 0.0)
        Climber.staticsLifted = SmartDashboard.getBoolean("static", false)
    }

    override fun end(interrupted: Boolean) {
        Climber.leftWinch.voltage = 0.0
        Climber.rightWinch.voltage = 0.0
        Climber.leftExtendable.voltage = 0.0
        Climber.rightExtendable.voltage = 0.0
    }
}
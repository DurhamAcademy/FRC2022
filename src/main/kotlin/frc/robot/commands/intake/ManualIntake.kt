package frc.robot.commands.intake

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Intaker

object ManualIntake : CommandBase() {
    init {
        addRequirements(Intaker)
    }

    override fun initialize() {
        SmartDashboard.putBoolean("deployed", Intaker.deployed)
        SmartDashboard.putNumber("intake percent", 0.0)
    }

    override fun execute() {
        Intaker.deployed = SmartDashboard.getBoolean("deployed", Intaker.deployed)
        Intaker.intakeMotor.percent = SmartDashboard.getNumber("intake percent", 0.0)
    }

    override fun end(interrupted: Boolean) {
        Intaker.intakeMotor.stop()
    }
}
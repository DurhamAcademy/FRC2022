package frc.robot.commands.intake

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Intaker

object ManualIntake : CommandBase() {
    const val deploymentString = "deployed"
    const val intakeString = "intake percent"
    init {
        addRequirements(Intaker)
        SmartDashboard.putBoolean(deploymentString, Intaker.deployed)
        SmartDashboard.putNumber(intakeString, 0.8)
    }

    override fun execute() {
        Intaker.deployed = SmartDashboard.getBoolean(deploymentString, Intaker.deployed)
        Intaker.intakeMotor.percent = SmartDashboard.getNumber(intakeString, 0.0)
    }

    override fun end(interrupted: Boolean) {
        Intaker.intakeMotor.stop()
    }
}
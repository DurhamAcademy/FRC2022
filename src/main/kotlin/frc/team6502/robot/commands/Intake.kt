package frc.team6502.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.Constants
import frc.team6502.robot.subsystems.Intaker

object Intake : CommandBase() {
    init {
        addRequirements(Intaker)
    }

    override fun initialize() {
        Intaker.deployed = true
        Intaker.intakeMotor.percent = Constants.INTAKE_PERCENT
    }

    override fun end(interrupted: Boolean) {
        Intaker.deployed = false
        Intaker.intakeMotor.stop()
    }
}
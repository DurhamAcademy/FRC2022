package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Intaker

/**
 * Deploys and activates the intake
 */
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
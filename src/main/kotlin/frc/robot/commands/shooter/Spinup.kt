package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Shooter

object Spinup : CommandBase() {
    init {
        addRequirements(Shooter)
    }

    override fun execute() {
        Shooter.update()
    }
}
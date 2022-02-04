package frc.team6502.robot.commands.climb

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team6502.robot.subsystems.Climber

class ToggleArmLift : InstantCommand() {
    override fun execute() {
        Climber.armsLifted = !Climber.armsLifted
    }
}
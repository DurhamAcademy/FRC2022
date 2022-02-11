package frc.robot.commands.climb

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.kyberlib.command.Debug
import frc.kyberlib.command.LogMode
import frc.robot.subsystems.Climber

/**
 * Toggles whether the static arms are lifted
 */
class ToggleArmLift : InstantCommand() {
    override fun execute() {
        Climber.armsLifted = !Climber.armsLifted
    }
}
package frc.robot.commands.climb

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.Climber

/**
 * Toggles whether the static arms are lifted
 */
class ToggleArmLift : InstantCommand() {
    override fun execute() {
        Climber.staticsLifted = !Climber.staticsLifted
    }
}
package frc.robot.commands.climb

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.inches
import frc.robot.subsystems.Climber

object PrepareClimb : CommandBase() {
    init {
        addRequirements(Climber)
    }

    override fun initialize() {
        Climber.extension = 17.5.inches
        Climber.armsLifted = false
    }

    override fun end(interrupted: Boolean) {
        if(!interrupted) Climber.extension = 0.inches
    }
}
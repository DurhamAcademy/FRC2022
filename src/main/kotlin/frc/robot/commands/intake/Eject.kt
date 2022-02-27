package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.rotationsPerSecond
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Turret

object Eject : CommandBase() {
    /// TODO: 2/25/22 add Eject Code
    override fun initialize() {
        addRequirements(Turret);
    }
    override fun execute() {
        if (Turret.turret.velocity <= 7.rotationsPerSecond)
            Turret.turret.velocity = 7.rotationsPerSecond

    }

    override fun end(interrupted: Boolean) {

    }
}
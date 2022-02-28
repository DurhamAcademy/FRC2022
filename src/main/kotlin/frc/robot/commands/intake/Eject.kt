package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.rotationsPerSecond
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Turret

object Eject : CommandBase() {
    /// TODO: 2/25/22 add Eject Code
    override fun initialize() {
        addRequirements(Shooter)
        addRequirements(Turret)
        addRequirements(Conveyor)
        beforeStarting(LoadEject)
    }

    override fun execute() {
        

    }

    override fun end(interrupted: Boolean) {

    }
}
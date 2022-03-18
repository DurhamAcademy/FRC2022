package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.rotationsPerSecond
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.Shooter

object Eject : CommandBase() {
    init {
        addRequirements(Shooter, Conveyor)
    }

    override fun initialize() {
        Shooter.targetVelocity = 4.rotationsPerSecond
        Conveyor.feed()
    }


    override fun execute() {
        Shooter.update()
    }

    override fun end(interrupted: Boolean) {
        Shooter.stop()
        Conveyor.stop()
    }
}

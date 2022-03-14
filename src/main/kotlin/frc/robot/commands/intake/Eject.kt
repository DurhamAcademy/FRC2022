package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.rotationsPerSecond
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.Shooter

object Eject : CommandBase() {
    init {
        addRequirements(Shooter)
    }
    override fun initialize() {
        Shooter.targetVelocity = 7.rotationsPerSecond
    }


    override fun execute() {
        Shooter.update()
        if(Shooter.ready) Feed.schedule()
    }

    override fun end(interrupted: Boolean) {
        Shooter.stop()
    }
}

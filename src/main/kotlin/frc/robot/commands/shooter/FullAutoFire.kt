package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Game
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.ShooterStatus
import frc.robot.subsystems.Turret

/**
 * Always firing when the flywheel is up to speed
 * Used for Autos that just don't stop.
 * This is not recommended for actual use I just got bored
 */
class FullAutoFire : CommandBase() {
    init {
        addRequirements(Shooter, Conveyor)
    }

    override fun execute() {
        Shooter.update()
        if ((Turret.targetVisible || Shooter.status == ShooterStatus.SHOT)) {

            // if the turret is on target
            if (Turret.ready && Shooter.ready) {
                Shooter.status = ShooterStatus.SHOT
                Conveyor.feed()
            }
            else {
                Shooter.status = ShooterStatus.SPINUP
                Conveyor.stop()
            }
        }
    }

    override fun end(interrupted: Boolean) {
        Shooter.stop()
    }

    override fun isFinished(): Boolean = Game.OPERATED
}
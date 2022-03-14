package frc.robot.commands.shooter

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.*
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.intake.Feed
import frc.robot.commands.intake.Idle
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.ShooterStatus
import frc.robot.subsystems.Turret

class AutoShot : CommandBase() {
    init {
        addRequirements(Shooter, Drivetrain)
    }

    override fun initialize() {
        shootingTimer.reset()
        Drivetrain.stop()
    }

    val shootingTimer = Timer()

    override fun execute() {
        if ((Turret.targetVisible || Shooter.status == ShooterStatus.SHOT)) {
            Shooter.update()

            // if the turret is on target
            if (Turret.ready && Shooter.ready) {
                shootingTimer.start()
                Shooter.status = ShooterStatus.SHOT
                Feed.schedule()
            }
            else {
                Shooter.status = ShooterStatus.SPINUP
            }
        }
    }

    override fun isFinished(): Boolean = shootingTimer.hasElapsed(2.0)
}
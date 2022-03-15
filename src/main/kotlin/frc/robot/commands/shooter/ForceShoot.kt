package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.math.units.extensions.radiansPerSecond
import frc.robot.Constants
import frc.robot.commands.intake.Feed
import frc.robot.subsystems.*

/**
 * Bypass the turret restrictions and forces the robot to shoot
 */
object ForceShoot : CommandBase() {
    init {
        addRequirements(Shooter, Conveyor)
    }

    override fun initialize() {
        Feed.schedule()
        Shooter.status = ShooterStatus.FORCE_SHOT
    }

    override fun execute() {
        Debug.log("Force Shoot", "execute", level=DebugFilter.Low)

        if (Shooter.ready) {
            Shooter.update()
        }
        else {
            Shooter.flywheel.percent = 0.5
        }
    }

    override fun end(interrupted: Boolean) {
        Shooter.stop()
    }

}
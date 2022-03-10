package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.centimeters
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.math.units.extensions.radiansPerSecond
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.intake.Feed
import frc.robot.commands.intake.Idle
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
        if ((Turret.targetVisible || Shooter.status == ShooterStatus.SHOT)) {
            val dis = if (Turret.targetVisible) Shooter.targetDistance!!.meters else RobotContainer.navigation.position.getDistance(
                Constants.HUB_POSITION)
            val targetFlywheelVelocity = Constants.FLYWHEEL_INTERPOLATOR.calculate(dis)!!.radiansPerSecond
            val targetHoodAngle = Constants.HOODANGLE_INTERPOLATOR.calculate(dis)!!

            // set the positions/velocities to the motors
            Shooter.flywheelMaster.velocity = targetFlywheelVelocity
            Shooter.hoodDistance = (targetHoodAngle / 10).centimeters

            // if the turret is on target
            if (Turret.readyToShoot && Shooter.flywheelMaster.velocityError < Constants.SHOOTER_VELOCITY_TOLERANCE && Shooter.hood.atSetpoint) {
                Shooter.status = ShooterStatus.SHOT
                Feed.schedule()
            }
            else {
                Idle.execute()
                Shooter.status = ShooterStatus.SPINUP
            }
        }
    }

    override fun isFinished(): Boolean = Game.OPERATED
}
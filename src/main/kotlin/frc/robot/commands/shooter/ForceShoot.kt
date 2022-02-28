package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugLevel
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
        addRequirements(Shooter)
    }

    override fun initialize() {
        Feed.schedule()
        Shooter.status = ShooterStatus.FORCE_SHOT
    }

    override fun execute() {
        Debug.log("Force Shoot", "execute", level=DebugLevel.LowPriority)

        if (Turret.targetVisible && Shooter.flywheelMaster.percent > 0.1) {
            val dis = if (Turret.targetVisible) Shooter.targetDistance!!.meters else Shoot.prevDistance
            val parallelSpeed = Drivetrain.polarSpeeds.dr
            Shoot.prevDistance = dis

            // calculate values given the current distance from the hub
            val targetFlywheelVelocity = Constants.FLYWHEEL_INTERPOLATOR.calculate(dis)!!.radiansPerSecond
            val targetTopWheelVelocity = Constants.TOPWHEEL_INTERPOLATOR.calculate(dis)!!.radiansPerSecond
            val targetHoodAngle = Constants.HOODANGLE_INTERPOLATOR.calculate(dis)!!.degrees

            // set the positions/velocities to the motors
            Shooter.flywheelMaster.velocity = targetFlywheelVelocity
            Shooter.topShooter.velocity = targetTopWheelVelocity
            Shooter.hoodAngle = targetHoodAngle
        }
        else {
            Shooter.hood.stop()
            Shooter.topShooter.percent = 0.5
            Shooter.flywheelMaster.percent = 0.5
        }
        Conveyor.feed()
    }

}
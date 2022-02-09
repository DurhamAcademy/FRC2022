package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.SHOOTER_STATUS
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Turret
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.math.units.extensions.radiansPerSecond


/**
 * Default Shooter method to automatically shoot balls when ready
 */
object Shoot : CommandBase() {
    init {
        addRequirements(Conveyor, Shooter)
    }

    override fun execute() {
        // check if shooter should spin up
        if (Conveyor.good && !Turret.targetLost && false) {  // todo: setup test interpolator
            val dis = Shooter.targetDistance!!.meters

            // calculate values given the current distance from the hub
            val targetFlywheelVelocity = Constants.FLYWHEEL_INTERPOLATOR.calculate(dis)!!.radiansPerSecond
            val targetTopWheelVelocity = Constants.TOPWHEEL_INTERPOLATOR.calculate(dis)!!.radiansPerSecond
            val targetHoodAngle = Constants.HOODANGLE_INTERPOLATOR.calculate(dis)!!.degrees

            // set the positions/velocities to the motors
            Shooter.flywheelMaster.velocity = targetFlywheelVelocity
            Shooter.topShooter.velocity = targetTopWheelVelocity
            Shooter.hoodAngle = targetHoodAngle

            // todo: bad if this stutters
            // if the turret is on target
            if (Turret.readyToShoot && Shooter.flywheelMaster.velocityError < Constants.SHOOTER_VELOCITY_TOLERANCE) {
                // change leds to green or something
                Conveyor.feed()
                Shooter.status = SHOOTER_STATUS.AUTO_SHOT
            } else Shooter.status = SHOOTER_STATUS.SPINUP
        } else Shooter.status = SHOOTER_STATUS.IDLE
    }

    override fun isFinished(): Boolean = false
}
package frc.team6502.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.Constants
import frc.team6502.robot.subsystems.Conveyor
import frc.team6502.robot.subsystems.Shooter
import frc.team6502.robot.subsystems.Turret
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.meters
import kyberlib.math.units.extensions.radiansPerSecond

object Shoot : CommandBase() {
    init {
        addRequirements(Conveyor, Shooter)
    }

    override fun execute() {
        if (Conveyor.hasBalls) {
            val dis = Shooter.targetDistance.meters
            val targetFlywheelVelocity = Constants.FLYWHEEL_INTERPOLATOR.calculate(dis)!!.radiansPerSecond
            val targetTopWheelVelocity = Constants.TOPWHEEL_INTERPOLATOR.calculate(dis)!!.radiansPerSecond
            val targetHoodAngle = Constants.HOODANGLE_INTERPOLATOR.calculate(dis)!!.degrees

            Shooter.flywheelMaster.velocity = targetFlywheelVelocity
            Shooter.topShooter.velocity = targetTopWheelVelocity
            Shooter.hoodAngle = targetHoodAngle

            // todo: bad if this stutters
            if (Turret.readyToShoot && Shooter.flywheelMaster.velocityError < Constants.SHOOTER_VELOCITY_TOLERANCE) {
                // change leds to green or something
                Conveyor.feed()
            }
        }
    }
}
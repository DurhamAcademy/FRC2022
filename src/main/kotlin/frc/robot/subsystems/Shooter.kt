package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.math.units.extensions.AngularVelocity
import frc.kyberlib.motorcontrol.rev.KSparkMax

object Shooter: SubsystemBase() {
    private val rightShootermotor = KSparkMax(1, fake = true)
    private val leftShootermotor = KSparkMax(2, fake = true).apply { follow(rightShootermotor) }

    fun setVelocity(speed: AngularVelocity) {
        rightShootermotor.velocity = speed
    }



}
package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.motorcontrol.rev.KSparkMax

object Shooter: SubsystemBase() {
    private val rightShootermotor = KSparkMax(1)
    private val leftShootermotor = KSparkMax(2).apply { follow(rightShootermotor) }





}
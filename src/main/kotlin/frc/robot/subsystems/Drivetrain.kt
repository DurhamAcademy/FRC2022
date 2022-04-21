package frc.robot.subsystems

import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.mechanisms.drivetrain.DifferentialDriveTrain
import frc.kyberlib.mechanisms.drivetrain.dynamics.KDifferentialDriveDynamic
import frc.kyberlib.motorcontrol.rev.KSparkMax

object Drivetrain : DifferentialDriveTrain() {
    private val leftMotor1 = KSparkMax(12).apply {
        radius = 2.inches
        kP = 1.0

    }
    private val leftMotor2 = KSparkMax(15).apply { follow(leftMotor1) }
    private val rightMotor1 = KSparkMax(13).apply {
        copyConfig(leftMotor1)
    }
    private val rightMotor2 = KSparkMax(10).apply { follow(rightMotor1) }
    
    override val dynamics: KDifferentialDriveDynamic = KDifferentialDriveDynamic(leftMotor1, rightMotor1)
}


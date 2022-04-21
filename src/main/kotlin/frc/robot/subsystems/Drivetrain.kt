package frc.robot.subsystems

import frc.kyberlib.mechanisms.drivetrain.DifferentialDriveTrain
import frc.kyberlib.mechanisms.drivetrain.dynamics.KDifferentialDriveDynamic
import frc.kyberlib.motorcontrol.rev.KSparkMax

object Drivetrain : DifferentialDriveTrain() {
    private val leftMotor1 = KSparkMax(1)
    private val leftMotor2 = KSparkMax(2).apply { follow(leftMotor1) }
    private val rightMotor1 = KSparkMax(3)
    private val rightMotor2 = KSparkMax(4).apply { follow(rightMotor1) }
    
    override val dynamics: KDifferentialDriveDynamic = KDifferentialDriveDynamic(leftMotor1, rightMotor1)
}


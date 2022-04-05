package frc.kyberlib.tutorial.Tutorial3_Structure.subsystems

import frc.kyberlib.motorcontrol.Voltage
import frc.kyberlib.motorcontrol.rev.KSparkMax

object Drivetrain {
    val leftMotor = KSparkMax(0)
    val rightMotor = KSparkMax(1)
    fun drive(leftVoltage: Voltage, rightVoltage: Voltage) {
        leftMotor.voltage = leftVoltage
        rightMotor.voltage = rightVoltage
    }

}
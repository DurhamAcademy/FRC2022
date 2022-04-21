package frc.robot.subsystems

import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.math.units.extensions.millimeters
import frc.kyberlib.math.units.extensions.millimetersPerSecond
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.motorcontrol.servo.KLinearServo


object Aim : SubsystemBase() {
    private val aimMotor = KSparkMax(1, fake = true)
    private val act1 = KLinearServo(2, 100.millimeters, 20.millimetersPerSecond)
    private val act2 = KLinearServo(3, 100.millimeters, 20.millimetersPerSecond)

    private val hall1 = DigitalInput(1)
}
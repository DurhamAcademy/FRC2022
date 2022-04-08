package frc.robot.subsystems

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.pneumatics.KSolenoid

/**
 * Controls the intake mechanism of the robot
 */
object Intaker : SubsystemBase(), Debug {
    // deployment solenoids
    private val leftIntakeDeploy = KSolenoid(4, 5, fake = false)
    private val rightIntakeDeploy = KSolenoid(2, 3, fake = false)

    // public get/set for deploy status
    var deployed
        get() = leftIntakeDeploy.extended
        set(value) {
            leftIntakeDeploy.extended = value
            rightIntakeDeploy.extended = value
        }

    // motor controlling the intake speed
    val intakeMotor = KSparkMax(20).apply {
        identifier = "intake"
        kP = 1.0
        currentLimit = 40
        motorType = DCMotor.getNeo550(1)
        brakeMode = false
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "deployed" to deployed,
            "intake" to intakeMotor
        )
    }


}
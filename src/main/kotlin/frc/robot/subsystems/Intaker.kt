package frc.robot.subsystems

import edu.wpi.first.math.system.plant.DCMotor
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugLevel
import frc.kyberlib.command.KSubsystem
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.pneumatics.KSolenoid

/**
 * Controls the intake mechanism of the robot
 */
object Intaker : KSubsystem(), Debug {
    override val priority: DebugLevel = DebugLevel.LowPriority
    // deployment solenoids
    private val leftIntakeDeploy = KSolenoid(2, fake = true)
    private val rightIntakeDeploy = KSolenoid(3, fake = true)

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
        motorType = DCMotor.getNeo550(1)
    }

    override fun periodic() {
        debugDashboard()
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "deployed" to deployed,
            "intake" to intakeMotor
        )
    }


}
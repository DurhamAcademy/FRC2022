package frc.team6502.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import kyberlib.command.Debug
import kyberlib.motorcontrol.rev.KSparkMax
import kyberlib.pneumatics.KSolenoid

object Intaker  : SubsystemBase(), Debug {
    private val leftIntakeDeploy = KSolenoid(0, 0)
    private val rightIntakeDeploy = KSolenoid(0, 0)

    var deployed
        get() = leftIntakeDeploy.extended
        set(value) {
            leftIntakeDeploy.extended = value
            rightIntakeDeploy.extended = value
        }

    val intakeMotor = KSparkMax(0)

    // todo add ball sensors

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
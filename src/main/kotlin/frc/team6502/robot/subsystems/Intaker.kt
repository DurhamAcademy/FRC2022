package frc.team6502.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import kyberlib.command.Debug
import kyberlib.motorcontrol.KSimulatedESC
import kyberlib.motorcontrol.rev.KSparkMax
import kyberlib.pneumatics.FakeSolenoid
import kyberlib.pneumatics.KSolenoid

/**
 * Controls the intake mechanism of the robot
 */
object Intaker  : SubsystemBase(), Debug {
    // deployment solenoids
    private val leftIntakeDeploy = FakeSolenoid(0, 0)
    private val rightIntakeDeploy = FakeSolenoid(0, 0)

    // public get/set for deploy status
    var deployed
        get() = leftIntakeDeploy.extended
        set(value) {
            leftIntakeDeploy.extended = value
            rightIntakeDeploy.extended = value
        }

    // motor controlling the intake speed
    val intakeMotor = KSimulatedESC("sim")

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
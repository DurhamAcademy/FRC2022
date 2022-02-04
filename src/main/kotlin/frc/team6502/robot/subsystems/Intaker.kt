package frc.team6502.robot.subsystems

import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team6502.robot.Constants
import kyberlib.command.Debug
import kyberlib.motorcontrol.rev.KSparkMax

object Intaker  : SubsystemBase(), Debug {
    private val leftIntakeDeploy = Solenoid(0, 0)
    private val rightIntakeDeploy = Solenoid(0, 0)

    var deployed
        get() = leftIntakeDeploy.get()
        set(value) {
            leftIntakeDeploy.set(value)
            rightIntakeDeploy.set(value)
        }

    val intakeMotor = KSparkMax(0)

    fun activate() {
        deployed = true
        intakeMotor.percent = Constants.INTAKE_PERCENT // todo: test this
    }

    fun deactivate() {
        deployed = false
        intakeMotor.stop()
    }

    // todo add ball sensors

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "deployed" to deployed,
            "intake" to intakeMotor
        )
    }


}
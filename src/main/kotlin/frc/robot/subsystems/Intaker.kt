package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Solenoid
import frc.kyberlib.command.Debug
import frc.kyberlib.motorcontrol.rev.KSparkMax

/**
 * Controls the intake mechanism of the robot
 */
object Intaker  : SubsystemBase(), Debug {
    // deployment solenoids
    private val leftIntakeDeploy = Solenoid(PneumaticsModuleType.CTREPCM, 2)
    private val rightIntakeDeploy = Solenoid(PneumaticsModuleType.CTREPCM, 3)

    // public get/set for deploy status
    var deployed
        get() = leftIntakeDeploy.get()
        set(value) {
            leftIntakeDeploy.set(value)
            rightIntakeDeploy.set(value)
        }

    // motor controlling the intake speed
    val intakeMotor = KSparkMax(0)

    // TODO add ball sensors

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
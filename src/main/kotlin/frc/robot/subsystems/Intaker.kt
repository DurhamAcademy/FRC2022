package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.PneumaticsModuleType
import frc.kyberlib.command.Debug
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.pneumatics.KSolenoid

/**
 * Controls the intake mechanism of the robot
 */
object Intaker  : SubsystemBase(), Debug {
    // deployment solenoids
    private val leftIntakeDeploy = KSolenoid(PneumaticsModuleType.REVPH, 0)  // todo: check if these configs are right
    private val rightIntakeDeploy = KSolenoid(PneumaticsModuleType.REVPH, 0)

    // public get/set for deploy status
    var deployed
        get() = leftIntakeDeploy.extended
        set(value) {
            leftIntakeDeploy.extended = value
            rightIntakeDeploy.extended = value
        }

    // motor controlling the intake speed
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
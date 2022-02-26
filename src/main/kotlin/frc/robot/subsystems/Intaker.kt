package frc.robot.subsystems

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.math.units.extensions.radiansPerSecond
import frc.kyberlib.motorcontrol.KSimulatedESC
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.pneumatics.KSolenoid

/**
 * Controls the intake mechanism of the robot
 */
object Intaker  : SubsystemBase(), Debug {
    init {
        log("init")
    }
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
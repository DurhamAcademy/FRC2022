package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.math.units.extensions.radiansPerSecond
import frc.kyberlib.motorcontrol.KSimulatedESC
import frc.kyberlib.motorcontrol.rev.KSparkMax

/**
 * Controls the intake mechanism of the robot
 */
object Intaker  : SubsystemBase(), Debug {
    init {
        log("init")
    }
    // deployment solenoids
//    private val leftIntakeDeploy = Solenoid(PneumaticsModuleType.CTREPCM, 2)
//    private val rightIntakeDeploy = Solenoid(PneumaticsModuleType.CTREPCM, 3)

    // public get/set for deploy status
    var deployed
        get() = false//leftIntakeDeploy.get()
        set(value) {
//            leftIntakeDeploy.set(value)
//            rightIntakeDeploy.set(value)
        }

    // motor controlling the intake speed
    val intakeMotor = KSparkMax(20).apply {
        kP = 1.0
        radius = 1.inches
    }

    init {
        SmartDashboard.putData("intake", intakeMotor)
//        intakeMotor.velocity = 1.radiansPerSecond
    }

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
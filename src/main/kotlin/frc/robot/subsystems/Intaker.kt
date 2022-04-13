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
    private val deployer = KSolenoid(4, 5, fake = false)
    private val deployFolower = KSolenoid(2, 3, fake = false).apply { follow(deployer) }

    var deployed
        get() = deployer.extended
        set(value) { deployer.extended = value}

    // motor controlling the intake speed
    val intakeMotor = KSparkMax(20).apply {
        identifier = "intake"
        currentLimit = 40
        brakeMode = false
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "deployed" to deployer.extended,
            "intake" to intakeMotor
        )
    }


}
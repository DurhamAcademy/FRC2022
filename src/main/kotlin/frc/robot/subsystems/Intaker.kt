package frc.robot.subsystems

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.pneumatics.KSolenoid
import frc.robot.Constants

/**
 * Controls the intake mechanism of the robot
 */
object Intaker : SubsystemBase(), Debug {
    // deployment solenoids
    private val deployer = KSolenoid(4, 5, fake = false)
    private val deployFolower = KSolenoid(2, 3, fake = false).apply { follow(deployer) }

    // motor controlling the intake speed
    private val intakeMotor = KSparkMax(20).apply {
        identifier = "intake"
        currentLimit = 40
        brakeMode = false
    }

    var deployed
        get() = deployer.extended
        set(value) { deployer.extended = value}

    fun intake(percent: Double = Constants.INTAKE_PERCENT) {
        intakeMotor.percent = percent
    }

    fun stop() {
        intakeMotor.stop()
    }
}
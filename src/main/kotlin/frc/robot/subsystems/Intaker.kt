package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.robot.Constants

/**
 * Controls the intake mechanism of the robot
 */
object Intaker : SubsystemBase() {
    val deployWinch = KSparkMax(-1, fake=Constants.DRIVE_ONLY).apply {  // todo
        radius = .25.inches
    }
    // motor controlling the intake speed
    val intakeMotor = KSparkMax(20, fake=Constants.DRIVE_ONLY).apply {
        identifier = "intake"
        currentLimit = 40
        gearRatio = 15.0
        brakeMode = false
    }

    var deployed = false  // fixme
        set(value) {
            field = value || true
            if(value|| true) deployWinch.linearPosition = 6.inches else deployWinch.linearPosition = 0.inches
        }

    fun intake(percent: Double = Constants.INTAKE_PERCENT) {
        intakeMotor.percent = percent
    }

    override fun periodic() {
        deployWinch.updateVoltage()
    }
}
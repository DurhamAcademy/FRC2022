// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.kyberlib.motorcontrol.characterization

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.auto.Navigator
import frc.kyberlib.math.filters.Differentiator
import frc.kyberlib.math.units.extensions.radians
import frc.kyberlib.math.units.extensions.rotations
import frc.kyberlib.math.units.extensions.rotationsPerSecond
import frc.kyberlib.motorcontrol.KMotorController
import kotlin.math.roundToLong

class Characterization(private vararg val motors: KMotorController, private val drivetrain: Boolean = false) : CommandBase() {
    private var startTime = 0.0
    private var data: String = ""

    // Called when the command is initially scheduled.
    override fun initialize() {
        SmartDashboard.putBoolean("SysIdOverflow", false)
        SmartDashboard.putString("SysIdTelemetry", "")
        startTime = Timer.getFPGATimestamp()
    }

    private val headingDiff = Differentiator()
    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        val timestamp = Timer.getFPGATimestamp()

        // Check if running the correct test
        val test = SmartDashboard.getString("SysIdTest", "")
        val correctTest: Boolean = if (drivetrain) {
            test == "Drivetrain" || test == "Drivetrain (Angular)"
        } else {
            test == "Arm" || test == "Elevator" || test == "Simple"
        }
        SmartDashboard.putBoolean("SysIdWrongMech", !correctTest)

        // Wrong test, prevent movement
        if (!correctTest) {
            motors.forEach { it.stop() }
            return
        }

        // Calculate voltage
        val testType = SmartDashboard.getString("SysIdTestType", "")
        val voltageCommand = SmartDashboard.getNumber("SysIdVoltageCommand", 0.0)
        val rotate = SmartDashboard.getBoolean("SysIdRotate", false)
        val baseVoltage: Double = when (testType) {
            "Quasistatic" -> voltageCommand * (timestamp - startTime)
            "Dynamic" -> voltageCommand
            else -> 0.0
        }
        val primaryVoltage = baseVoltage * if (rotate) -1 else 1
        val secondaryVoltage = baseVoltage

        // Set output and get new data
        if (drivetrain) {
            val leftMotor = motors[0]
            val rightMotor = motors[1]
            leftMotor.voltage = primaryVoltage
            rightMotor.voltage = primaryVoltage
            data += "$timestamp,$primaryVoltage,$secondaryVoltage," +
                    "${leftMotor.position.rotations},${rightMotor.position.rotations}," +
                    "${leftMotor.velocity.rotationsPerSecond}, ${rightMotor.velocity.rotationsPerSecond}," +
                    "${Navigator.instance!!.heading.radians},${headingDiff.calculate(Navigator.instance!!.heading.radians)},"
        } else {
            motors.forEach { it.voltage = primaryVoltage }
            val definingMotor = motors.first()
            data += "$timestamp,$primaryVoltage,${definingMotor.position.rotations},${definingMotor.velocity.rotationsPerSecond},"
        }
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        if (data.isNotEmpty()) {
            SmartDashboard.putString(
                "SysIdTelemetry",
                data.substring(0, data.length - 1)
            )
            println("Saved ${(data.length / 1024.0).roundToLong().toString()} KB of data.")
        } else {
            println("No data to save. Something's gone wrong here...")
        }
        motors.forEach {
            it.stop()
        }
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        motors.forEach {
            if(it.position > it.maxPosition || it.position < it.minPosition) return true
        }
        return false
    }
}
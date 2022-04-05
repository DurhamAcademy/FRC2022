package frc.kyberlib.motorcontrol.characterization

import frc.kyberlib.math.Polynomial
import kotlin.math.absoluteValue

object CharacterizationAnalyser {
    fun analyze(telemetry: String) {
        val data = telemetry.split(',').map { it.toDouble() }
        var index = 0
        val voltage = mutableListOf<Double>()
        val output = mutableListOf<Double>()
        val output2 = mutableListOf<Double>()
        while(index < data.size) {
            if(CharacterizationUtil.drivetrain) {
                val time = data[index]
                val volt = data[index + 1]
                val volt2 = data[index + 2]
                val position = data[index+3]
                val position2 = data[index+4]
                val velocity = data[index+5]
                val velocity2 = data[index+6]
                val heading = data[index+7]
                val spinRate = data[index+8]
                index += 9

                voltage.add(volt.absoluteValue)
                if(CharacterizationUtil.type == "Drivetrain (Angular)") {
                    output.add(spinRate.absoluteValue)
                } else {
                    output.add(velocity)
                    output2.add(velocity2)
                }
            }
            else {
                val time = data[index]
                val volt = data[index + 1]
                val position = data[index+2]
                val velocity = data[index+3]
                voltage.add(volt)
                output.add(velocity)

                index += 4
            }
        }

        if(CharacterizationUtil.drivetrain) {
            analyze(voltage.toDoubleArray(), output.toDoubleArray())
            analyze(voltage.toDoubleArray(), output2.toDoubleArray())
        }
    }

    fun analyze(inputs: DoubleArray, outputs: DoubleArray) {
        println(Polynomial.regress(inputs, outputs))
    }
}
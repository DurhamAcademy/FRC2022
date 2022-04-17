package frc.kyberlib.motorcontrol.characterization

import edu.wpi.first.math.filter.LinearFilter
import frc.kyberlib.math.Polynomial
import frc.kyberlib.toMap
import kotlinx.serialization.ExperimentalSerializationApi
import kotlinx.serialization.json.*
import java.io.File
import kotlin.math.absoluteValue
import kotlin.math.pow

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

    fun analyze(voltages: DoubleArray, velocities: DoubleArray) {
        val p = Polynomial.regress(voltages, velocities)
        println(p)
    }

    @OptIn(ExperimentalSerializationApi::class)
    fun analyze(file: File) {
        val data = Json.decodeFromStream<JsonObject>(file.inputStream())
        val quasForward = data["slow-forward"]!!.jsonArray
        val voltages = mutableListOf<Double>()
        val velocities = mutableListOf<Double>()
        val accelerations = mutableListOf<Double>()
        var prevTime = -1.0
        var prevVel = -1.0
        quasForward.forEach { datum ->
            val arr = datum.jsonArray
            val time = arr[0].jsonPrimitive.double
            val leftVoltage = arr[1].jsonPrimitive.double
            val leftVelocity = arr[6].jsonPrimitive.double
            if(prevTime == -1.0) prevVel = leftVoltage
            val leftAcc = (leftVelocity - prevVel) / (time - prevTime)
            voltages.add(leftVoltage)
            velocities.add(leftVelocity)
            accelerations.add(leftAcc)
            prevTime = time
        }
        analyze(velocities.toDoubleArray(), voltages.toDoubleArray())
        voltages.clear()
        velocities.clear()
        accelerations.clear()
        val dyForward = data["fast-forward"]!!.jsonArray
        val filter = LinearFilter.movingAverage(5)
        dyForward.forEach { datum ->
            val arr = datum.jsonArray
            val time = arr[0].jsonPrimitive.double
            val leftVoltage = arr[1].jsonPrimitive.double
            val leftVelocity = arr[5].jsonPrimitive.double
            if(prevTime == -1.0) prevVel = leftVoltage
            val leftAcc = filter.calculate((leftVelocity - prevVel) / (time - prevTime))
            voltages.add(leftVoltage - 2.47 * leftVelocity)
            velocities.add(leftVelocity)
            accelerations.add(leftAcc)
            prevVel = leftVelocity
            prevTime = time
        }
        val a = voltages.slice(0..(voltages.size-11)).toDoubleArray()
        val b = accelerations.slice(10 until (accelerations.size)).toDoubleArray()
        println("${a.size}, ${b.size}")
        analyze(a, b)
    }
}

fun main() {
    CharacterizationAnalyser.analyze(File("calibration/sysid_data20220414-190720.json"))
}
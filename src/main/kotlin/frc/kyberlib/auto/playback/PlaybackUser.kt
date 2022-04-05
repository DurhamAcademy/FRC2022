package frc.kyberlib.auto.playback

import frc.kyberlib.command.Game
import frc.kyberlib.input.KAxis
import frc.kyberlib.input.KButton
import frc.kyberlib.input.KController
import frc.kyberlib.math.units.extensions.seconds
import java.io.File

class PlaybackUser(file: File) {
    val length = file.length().toInt()
    val times = arrayOfNulls<Double>(length)
    val buttons = arrayOfNulls<BooleanArray>(length)
    val axises = arrayOfNulls<DoubleArray>(length)

    init {
        var index = 0
        file.forEachLine {
            val parts = it.split(';')
            times[index] = parts[0].toDouble()
            buttons[index] = parseBoolList(parts[1])
            axises[index] = parseDoubleList(parts[2])
            index++
        }
    }

    var startTime = Game.time
    fun start() {
        startTime = Game.time
        KController.sim = true
    }

    fun update() {
        val timeIndex = times.binarySearch((Game.time - startTime).seconds)
        if (timeIndex < 0) {
            println("done")
            end()
            return
        }
        KButton.all.forEachIndexed { index, kButton ->
            kButton.simValue = buttons[timeIndex]!![index]
        }
        KAxis.all.forEachIndexed { index, kAxis ->
            kAxis.simVal = axises[timeIndex]!![index]
        }
    }

    private fun parseBoolList(string: String): BooleanArray {
        return (string.removeSurrounding("[", "]")
                .takeIf(String::isNotEmpty) // this handles the case of "[]"
                ?.split(", ")?.map { it.toBooleanStrict() }
                ?: emptyList()).toBooleanArray() // in the case of "[]"
    }

    private fun parseDoubleList(string: String): DoubleArray {
        return (string.removeSurrounding("[", "]")
                .takeIf(String::isNotEmpty) // this handles the case of "[]"
                ?.split(", ")?.map { it.toDouble() }
                ?: emptyList()).toDoubleArray() // in the case of "[]"
    }

    fun end() {
        KController.sim = false
    }
}
package frc.kyberlib.auto.playback

import frc.kyberlib.command.Game
import frc.kyberlib.input.KAxis
import frc.kyberlib.input.KButton
import frc.kyberlib.input.KController
import frc.kyberlib.math.units.extensions.seconds
import java.io.File

class PlaybackUser(file: File) {
    val length = file.reader().readLines().size
    val times = arrayOfNulls<Double>(length)
    val buttons = arrayOfNulls<BooleanArray>(length)
    val axises = arrayOfNulls<DoubleArray>(length)

    init {
        var index = 0
        file.forEachLine {
            val parts = it.split(';')
            if(parts.size != 3) {
                times[index] = times[index-1]
                buttons[index] = buttons[index-1]
                axises[index] = axises[index-1]
            }
            else {
                times[index] = parts[0].toDouble()
                buttons[index] = parseBoolList(parts[1])
                axises[index] = parseDoubleList(parts[2])
            }
            index++
        }
    }

    var startTime = Game.time
    fun start() {
        startTime = Game.time
        KController.sim = true
    }

    fun update() {
        val timeIndex = -times.binarySearch(Game.matchTime.seconds) - 1
        if (timeIndex <= 0 || timeIndex == times.size) {
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
                ?.split(", ")?.map { it.toBoolean() }
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


fun main() {
    val x = listOf(1.0, 1.5)
    println(x.toString())
}
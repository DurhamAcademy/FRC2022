package frc.kyberlib.auto.playback

import frc.kyberlib.command.Game
import frc.kyberlib.input.KAxis
import frc.kyberlib.input.KButton
import frc.kyberlib.math.units.extensions.seconds
import java.io.File

class PlaybackRecorder() {
    private val times = mutableListOf<Double>()
    private val buttons = mutableListOf<BooleanArray>()
    private val axises = mutableListOf<DoubleArray>()

    var startTime = Game.time
    fun reset() {
        startTime = Game.time
        times.clear()
        buttons.clear()
        axises.clear()
    }
    fun update() {
        val time = (Game.time - startTime).seconds
        times.add(time)
        buttons.add(KButton.all.map { it.get() }.toBooleanArray())
        axises.add(KAxis.all.map{it.value}.toDoubleArray())
    }

    fun save(file: File) {
        val writer = file.writer()
        times.zip(buttons).zip(axises).forEach {
            val time = it.first.first
            val buttons = it.first.second
            val axises = it.second
            writer.appendLine("$time;$buttons;$axises")
        }
    }
}
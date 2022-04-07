package frc.kyberlib.lighting

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.util.Color8Bit
import frc.kyberlib.command.Game

class KLEDStrip(port: Int, private val length: Int) {

    companion object {
        private var singleton: KLEDStrip? = null
    }

    private val addressableLED = AddressableLED(port)
    private val regions = arrayListOf<KLEDRegion>()

    private val startTime = Game.time

    init {
        require(singleton == null) { "Only one LED strip may be used at once" }
        addressableLED.setLength(length)
        addressableLED.start()
    }

    operator fun plusAssign(other: KLEDRegion) {
        require(other.end > other.start)
        require(other.start >= 0)
        require(other.end <= length)

        regions.add(other)
    }

    fun update() {
        val buffer = AddressableLEDBuffer(length)
        val dt = Game.time - startTime
        val mutableBuffer = KLEDRegion.composite(length, dt, regions).map {
            it.gammaCorrect()
        }

        for (i in mutableBuffer.indices) {
            buffer.setLED(i, Color8Bit(mutableBuffer[i].red, mutableBuffer[i].green, mutableBuffer[i].blue))
        }

        addressableLED.setData(buffer)
    }
}

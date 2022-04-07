package frc.kyberlib.lighting.animations

import frc.kyberlib.math.units.extensions.Time
import frc.kyberlib.math.units.extensions.seconds
import java.awt.Color

class AnimationRGBWave(private val cycles: Double = 1.0, val secondsPerMovement: Time = .2.seconds, val reversed: Boolean = false, enableTransparency: Boolean = false, condition: ()->Boolean = { true }) : LEDAnimation(condition, enableTransparency) {

    fun constructInitialBuffer(length: Int): MutableList<Color> {

        return if (cycles >= 1) {
            Array<Color>(length) {
                val hue = (it % (length / cycles)) / (length / cycles)
                Color.getHSBColor(hue.toFloat(), 1F, 1F)
            }.asList().toMutableList()
        } else {
            Array<Color>((length / cycles).toInt()) {
                val hue = it / (length / cycles)
                Color.getHSBColor(hue.toFloat(), 1F, 1F)
            }.asList().toMutableList()
        }
    }

    override fun getBuffer(time: Time, length: Int): List<Color> {
        val b = constructInitialBuffer(length)

        for (i in 0 until (time / secondsPerMovement).toInt() % b.size) {
            b.add(0, b.removeAt(b.size - 1))
        }

        if (reversed) b.reverse()

        return b.take(length)
    }
}

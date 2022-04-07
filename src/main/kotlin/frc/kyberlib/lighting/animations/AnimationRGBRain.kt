package frc.kyberlib.lighting.animations

import frc.kyberlib.math.units.extensions.Time
import java.awt.Color
import kotlin.math.ceil
import kotlin.math.pow

class AnimationRGBRain(private val cycles: Double = 1.0, private val dropLength: Int, val secondsPerMovement: Time, val reversed: Boolean = false, enableTransparency: Boolean = false, condition: ()->Boolean = { true }) : LEDAnimation(condition, enableTransparency) {

    private fun constructInitialBuffer(length: Int): MutableList<Color> {

        return Array<Color>(dropLength * ceil(length.toDouble() / dropLength).toInt()) {
            val alpha = ((1 + (it % dropLength)) / dropLength.toFloat()).pow(2)
            Color(1F, 1F, 1F, alpha)
        }.toMutableList()
    }

    private fun constructRGBBuffer(length: Int): MutableList<Color> {
        return if (cycles >= 1) {
            Array<Color>(length) {
                val hue = (it % (length / cycles)) / (length / cycles)
                Color.getHSBColor(1 - hue.toFloat(), 1F, 1F)
            }.asList().toMutableList()
        } else {
            Array<Color>((length / cycles).toInt()) {
                val hue = it / (length / cycles)
                Color.getHSBColor(1 - hue.toFloat(), 1F, 1F)
            }.asList().toMutableList()
        }
    }

    override fun getBuffer(time: Time, length: Int): List<Color> {
        val b = constructInitialBuffer(length)
        val rgb = constructRGBBuffer(length)

//        println(rgb.size)

        for (i in 0 until (time / secondsPerMovement).toInt() % (dropLength * rgb.size)) {
            b.add(0, b.removeAt(b.size - 1))
            rgb.add(0, rgb.removeAt(rgb.size - 1))
        }

        val trimmed = b.take(length).toMutableList()
        for (i in trimmed.indices) {
            trimmed[i] = Color((trimmed[i].red * rgb[i].red / 255.0).toInt(), (trimmed[i].green * rgb[i].green / 255.0).toInt(), (trimmed[i].blue * rgb[i].blue / 255.0).toInt(), trimmed[i].alpha)
        }

        if (reversed) {
            trimmed.reverse()
        }

        return trimmed
    }
}

package frc.kyberlib.lighting.animations

import frc.kyberlib.math.units.extensions.Time
import java.awt.Color
import kotlin.math.ceil
import kotlin.math.pow

class AnimationRain(private val color: Color, private val dropLength: Int, val secondsPerMovement: Time, val reversed: Boolean = false, enableTransparency: Boolean = false, condition: ()->Boolean = { true }) : LEDAnimation(condition, enableTransparency) {

    private fun constructInitialBuffer(length: Int): MutableList<Color> {

        return Array(dropLength * ceil(length.toDouble() / dropLength).toInt()) {
            val alpha = ((1 + (it % dropLength)) / dropLength.toFloat()).pow(2)
            Color(color.red / 255F, color.green / 255F, color.blue / 255F, alpha)
        }.toMutableList()
    }

    override fun getBuffer(time: Time, length: Int): List<Color> {
        val b = constructInitialBuffer(length)

        for (i in 0 until (time / secondsPerMovement).toInt() % dropLength) {
            b.add(0, b.removeAt(b.size - 1))
        }

        if (reversed) b.reverse()

        return b.take(length)
    }
}

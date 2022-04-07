package frc.kyberlib.lighting.animations

import frc.kyberlib.math.units.extensions.Time
import java.awt.Color
import kotlin.math.PI
import kotlin.math.roundToInt
import kotlin.math.sin

class AnimationBlink(val color: Color, val cycle: Time, enableTransparency: Boolean = false, condition: ()->Boolean = { true }) : LEDAnimation(condition, enableTransparency) {

    override fun getBuffer(time: Time, length: Int): List<Color> {
        val brightness = ((time % cycle) / cycle).roundToInt()
        return Array(length) {
            Color(color.red / 255F, color.green / 255F, color.blue / 255F, brightness.toFloat())
        }.toMutableList()
    }
}

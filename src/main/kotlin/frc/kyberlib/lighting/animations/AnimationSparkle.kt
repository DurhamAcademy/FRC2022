package frc.kyberlib.lighting.animations

import frc.kyberlib.math.randomizer
import frc.kyberlib.math.units.extensions.Time
import java.awt.Color

class AnimationSparkle(private val color: Color, enableTransparency: Boolean = false, condition: ()->Boolean = { true }) : LEDAnimation(condition, enableTransparency) {
    override fun getBuffer(time: Time, length: Int): List<Color> {
        return Array(length) {
            Color(color.red, color.green, color.blue, randomizer.nextInt(80) + 175)
        }.toMutableList()
    }
}
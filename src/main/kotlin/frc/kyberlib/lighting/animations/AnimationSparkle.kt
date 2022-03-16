package frc.kyberlib.lighting.animations

import frc.kyberlib.math.randomizer
import java.awt.Color

class AnimationSparkle(private val color: Color) : LEDAnimation() {
    override fun getBuffer(ticks: Int, length: Int): List<Color> {
        return Array(length) {
            Color(color.red, color.green, color.blue, randomizer.nextInt(80) + 175)
        }.toMutableList()
    }
}
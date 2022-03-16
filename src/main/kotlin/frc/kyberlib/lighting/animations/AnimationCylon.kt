package frc.kyberlib.lighting.animations

import java.awt.Color
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.sin

class AnimationCylon(private val color: Color, private val width: Int, private val ticksPerCycle: Int) : LEDAnimation() {
    override fun getBuffer(ticks: Int, length: Int): List<Color> {
        val center = (length-1) * (sin(2 * PI * (ticks % ticksPerCycle) / ticksPerCycle.toDouble()) + 1) / 2
        return Array(length) {
            Color(color.red / 255F, color.green / 255F, color.blue / 255F,  ((width - (center - it).absoluteValue) / width).coerceIn(0.0, 1.0).toFloat())
        }.toMutableList()
    }
}
package frc.kyberlib.lighting.animations

import frc.kyberlib.math.units.extensions.TAU
import java.awt.Color
import kotlin.math.absoluteValue
import kotlin.math.cos

class AnimationSilon(private val color: Color, private val width: Int, private val period: Int) : LEDAnimation() {
    val off = Color(0, 0, 0)
    override fun getBuffer(ticks: Int, length: Int): List<Color> {
        val center = cos(ticks/period* TAU)
        return Array(length) {
            if ((it-center).absoluteValue < width) color else off
        }.toMutableList()
    }
}
package frc.kyberlib.lighting.animations

import frc.kyberlib.math.units.extensions.Time
import java.awt.Color

abstract class LEDAnimation(val condition: ()->Boolean, val enableTransparency: Boolean) {
    abstract fun getBuffer(time: Time, length: Int): List<Color>
}

operator fun Color.times(mult: Double) = Color((red * mult).toInt(), (green * mult).toInt(), (blue * mult).toInt(), alpha)
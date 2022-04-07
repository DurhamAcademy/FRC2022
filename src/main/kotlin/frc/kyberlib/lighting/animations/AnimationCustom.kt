package frc.kyberlib.lighting.animations

import frc.kyberlib.math.units.extensions.Time
import java.awt.Color


class AnimationCustom(val buffer: (t: Time, l: Int)->List<Color>, condition: ()->Boolean, transparency: Boolean = false) : LEDAnimation(condition, transparency) {
    override fun getBuffer(time: Time, length: Int): List<Color> {
        return buffer(time, length)
    }
}
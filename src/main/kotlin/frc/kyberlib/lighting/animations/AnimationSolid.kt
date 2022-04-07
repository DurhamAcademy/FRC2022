package frc.kyberlib.lighting.animations

import frc.kyberlib.math.units.extensions.Time
import java.awt.Color

class AnimationSolid(val color: Color, enableTransparency: Boolean = false, condition: ()->Boolean = { true }) : LEDAnimation(condition, enableTransparency) {

    override fun getBuffer(time: Time, length: Int): List<Color> {
        return Array(length) {
            color
        }.toMutableList()
    }
}

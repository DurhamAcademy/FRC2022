package frc.kyberlib.lighting.animations

import frc.kyberlib.math.units.extensions.Time
import java.awt.Color

class AnimationRGBFade(val cycle: Time, enableTransparency: Boolean = false, condition: ()->Boolean = { true }) : LEDAnimation(condition, enableTransparency) {
    override fun getBuffer(time: Time, length: Int): List<Color> {
        return Array<Color>(length) {
            Color.getHSBColor(((time % cycle) / cycle).toFloat() % 1F, 1F, 1F)
        }.toMutableList()
    }
}

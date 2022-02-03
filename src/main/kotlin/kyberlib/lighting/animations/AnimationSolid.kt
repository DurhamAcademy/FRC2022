package kyberlib.lighting.animations

import java.awt.Color

class AnimationSolid(val color: Color) : LEDAnimation() {

    override fun getBuffer(ticks: Int, length: Int): List<Color> {
        return Array(length) {
            color
        }.toMutableList()
    }
}

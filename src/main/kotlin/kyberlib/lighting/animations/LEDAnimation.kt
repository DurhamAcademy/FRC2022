package kyberlib.lighting.animations

import java.awt.Color

abstract class LEDAnimation {
    abstract fun getBuffer(ticks: Int, length: Int): List<Color>
}

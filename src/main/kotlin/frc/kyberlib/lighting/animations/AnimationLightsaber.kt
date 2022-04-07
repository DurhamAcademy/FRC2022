package frc.kyberlib.lighting.animations

import frc.kyberlib.command.Game
import frc.kyberlib.math.randomizer
import frc.kyberlib.math.units.extensions.Time
import frc.kyberlib.math.units.extensions.seconds
import java.awt.Color

class AnimationLightsaber(val color: Color, transparency: Boolean= false, condition: ()->Boolean = {true}) : LEDAnimation(condition, transparency) {
    var startTime = -1.seconds
    var lastUpdate = Game.time

    private fun flicker(t: Time): Double {
        return 0.9 + randomizer.nextGaussian() * 0.1
    }

    private fun length(t: Time): Double {
        return t.seconds.coerceAtMost(1.0)
    }
    override fun getBuffer(time: Time, length: Int): List<Color> {
        if(startTime < 0.seconds || Game.time - lastUpdate > 3.seconds)
            startTime = time
        lastUpdate = time

        val dt = time - startTime
        val brightness = flicker(dt).coerceAtMost(1.0)
        val len = length(dt)
        return List<Color>(length) {index: Int -> if (index < length * len) color * brightness else Color.BLACK}
    }
}

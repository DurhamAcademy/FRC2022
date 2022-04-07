package frc.kyberlib.lighting

import frc.kyberlib.lighting.animations.LEDAnimation
import frc.kyberlib.math.units.extensions.Time
import java.awt.Color

class KLEDRegion(val start: Int, val end: Int, vararg val animations: LEDAnimation, val reversed: Boolean = false) {
    val length = end - start
    var transparent = true
    companion object {
        fun composite(length: Int, time: Time, regions: List<KLEDRegion>): Array<Color> {
            val mutableBuffer = Array<Color>(length) { Color.BLACK }

            for (region in regions) {
                val b = region.getBuffer(time) ?: continue
                for (i in b.indices) {
                    if (b[i].alpha < 255 && region.transparent) {
                        mutableBuffer[region.start + i] = Color(
                                (b[i].red / 255F) * (b[i].alpha / 255F) + (mutableBuffer[region.start + i].red / 255F) * (1 - b[i].alpha / 255F),
                                (b[i].green / 255F) * (b[i].alpha / 255F) + (mutableBuffer[region.start + i].green / 255F) * (1 - b[i].alpha / 255F),
                                (b[i].blue / 255F) * (b[i].alpha / 255F) + (mutableBuffer[region.start + i].blue / 255F) * (1 - b[i].alpha / 255F)
                        )
                    } else {
                        mutableBuffer[region.start + i] = Color(b[i].red, b[i].green, b[i].blue)
                    }
                }
            }
            return mutableBuffer
        }
    }

    fun getBuffer(time: Time): List<Color> {
        val mutableBuffer = Array<Color>(length) { Color.BLACK }
        animations.reversed().forEach { animation: LEDAnimation ->
            if(animation.condition()) {
                // get buffer
                val buffer = animation.getBuffer(time, length)
                val b = if(reversed) buffer.reversed() else buffer
                if (!animation.enableTransparency) {
                    return b.map { color ->
                        Color(
                            (color.red / 255F) * (color.alpha / 255F),
                            (color.green / 255F) * (color.alpha / 255F),
                            (color.blue / 255F) * (color.alpha / 255F)
                        )
                    }
                }

                // apply buffer
                for (i in b.indices) {
                    if (b[i].alpha < 255) {
                        mutableBuffer[start + i] = Color(
                            (b[i].red / 255F) * (b[i].alpha / 255F) + (mutableBuffer[start + i].red / 255F) * (1 - b[i].alpha / 255F),
                            (b[i].green / 255F) * (b[i].alpha / 255F) + (mutableBuffer[start + i].green / 255F) * (1 - b[i].alpha / 255F),
                            (b[i].blue / 255F) * (b[i].alpha / 255F) + (mutableBuffer[start + i].blue / 255F) * (1 - b[i].alpha / 255F)
                        )
                    } else {
                        mutableBuffer[start + i] = Color(b[i].red, b[i].green, b[i].blue)
                    }
                }
            }
        }
        return mutableBuffer.toList()
    }
}

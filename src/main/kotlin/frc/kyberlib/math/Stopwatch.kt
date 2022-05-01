package frc.kyberlib.math

import frc.kyberlib.math.units.extensions.Time
import frc.kyberlib.math.units.extensions.seconds

/**
 * Class to time how long things take
 */
class Stopwatch {
    val startTime = time
    var prevTime = startTime

    inline val time get() = (System.nanoTime() / 1e9).seconds
    inline val absoluteTime get() = time - startTime
    inline val loopTime get() = time - prevTime

    fun loop(): Time {
        val dt = time - prevTime
        prevTime = time
        return dt
    }
}
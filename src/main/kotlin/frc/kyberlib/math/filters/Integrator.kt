package frc.kyberlib.math.filters

import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.seconds

/**
 * Integrates a stream of data by time
 */
class Integrator : Filter {
    private var prevTime = 0.0
    private val dt: Double
        get() {
            val newTime = Game.time.seconds
            val change = newTime - prevTime
            prevTime = newTime
            return change
        }
    private var value = 0.0
    override fun calculate(d: Double): Double {
        // right-hand approximation
        value = if (prevTime != -1.0) value * dt else 0.0
        return get()
    }
    override fun get(): Double = value
}
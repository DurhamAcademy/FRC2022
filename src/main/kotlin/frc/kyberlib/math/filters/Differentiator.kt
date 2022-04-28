package frc.kyberlib.math.filters

import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.*

/**
 * Gets the rate of change of a stream of values
 */
class Differentiator : Filter {
    private var lastValue: Double? = null
    private var value = 0.0

    private var prevTime = 0.0
    private val dt: Double
        inline get() = Game.time.seconds - prevTime

    /**
     * Return the rate of change of value in units per second
     */
    override fun calculate(d: Double): Double {
        value = if (lastValue != null) (d - lastValue!!) / dt else 0.0
        lastValue = d
        prevTime = Game.time.seconds
        return value
    }

    fun calculate(angle: Angle): AngularVelocity = calculate(angle.radians).radiansPerSecond
    @JvmName("calculate1")
    fun calculate(dis: Length): LinearVelocity = calculate(dis.meters).metersPerSecond

    override fun get(): Double = value
}

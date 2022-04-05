package frc.kyberlib.math.filters

/**
 * Gets the rate of change of a stream of values
 */
class Differentiator : Filter() {
    private var lastValue: Double? = null
    private var value = 0.0

    private var prevTime = 0.0
    private val dt: Double
        inline get() = time - prevTime

    /**
     * Return the rate of change of value in units per second
     */
    override fun calculate(d: Double): Double {
        value = if (lastValue != null) (d - lastValue!!) / dt else 0.0
        lastValue = d
        prevTime = time
        return value
    }

//    inline fun <reified T: KUnitKey>calculate(unit: KUnit<T>) = KUnit<T>(calculate(unit.value))

    override fun get(): Double = value
}

package frc.kyberlib.math.filters

/**
 * Class that manipulates a incoming steam of doubles
 */
interface Filter {
    /**
     * Takes a new piece of data and outputs how it affected the filter
     */
    fun calculate(d: Double): Double

    /**
     * Get the current value of the filter without updating it
     */
    fun get(): Double
}
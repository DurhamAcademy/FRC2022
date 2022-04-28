package frc.kyberlib.math

/**
 * Linear Interpolator that will use a series of points to approximate a value
 */
class Interpolator(private val data: Map<Double, Double>) {
    private val keys = data.keys.toTypedArray()
    companion object {
        @JvmStatic
        fun main(args: Array<String>) {
            val test = Interpolator(
                mapOf(
                    0.0 to 1000.0,
                    1.0 to 1500.0,
                    2.0 to 2500.0,
                    3.0 to 5000.0,
                    4.0 to 6000.0
                )
            )
            check(-1.0, test)
            check(0.0, test)
            check(1.0, test)
            check(1.2, test)
            check(3.2, test)
            check(5.2, test)
        }

        private fun check(dis: Double, interpolator: Interpolator) {
            println("dis: $dis, val: ${interpolator.calculate(dis)}")
        }
    }

    operator fun get(x: Double) = calculate(x)
    /**
     * Approximate the value of x using the stored Data
     * @param x the value to use
     * @return an estimated y output
     */
    fun calculate(x: Double): Double {
        val i = index(x)
        if(i == -1) return data[keys.first()]!!  // datum below table
        if(i == -2) return data[keys.last()]!!  // datum above table
        val lowVal = data[keys[i]]!!
        val highVal = data[keys[i+1]]!!

        val alpha = (x-keys[i]) / (keys[i+1] - keys[i])  // percent distance from low to high
        return lowVal + alpha * (highVal - lowVal)  // weighted average
    }

    private fun index(x: Double): Int {
        keys.forEachIndexed { index, d ->
            if(x<d) return index - 1
        }
        return -2  // the condition if x is greater than all values
    }

}

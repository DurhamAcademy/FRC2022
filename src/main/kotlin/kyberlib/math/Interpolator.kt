package kyberlib.math

/**
 * Linear Interpolator that will use a series of points to approximate a value
 */
class Interpolator(private val data: Map<Double, Double>) {

    /**
     * Approximate the value of x using the stored Data
     * @param x the value to use
     * @return an estimated y output
     */
    fun calculate(x: Double): Double? {

        val nextHighest = getNext(x)
        val nextLowest = getPrevious(x)

        if(nextHighest == null || nextLowest == null) return null
        if(nextHighest.value == nextLowest.value) return nextLowest.value

        val alpha = (x - nextLowest.key) / (nextHighest.key - nextLowest.key)

        return nextLowest.value + alpha * (nextHighest.value - nextLowest.value)

    }

    /**
     * Get the next stored value after x
     * @param x the value to index from
     */
    private fun getNext(x: Double): Map.Entry<Double, Double>? {

        var nextHighest: Map.Entry<Double, Double>? = null

        for(k in data) {
            if(k.key >= x && ((nextHighest == null) || (k.key < nextHighest.key)))
                nextHighest = k
        }

        return nextHighest

    }

    /**
     * Get the nearest stored value before x
     * @param x the value to index from
     */
    private fun getPrevious(x: Double): Map.Entry<Double, Double>? {

        var nextLowest: Map.Entry<Double, Double>? = null

        for(k in data){
            if(k.key <= x && ((nextLowest == null) || (k.key > nextLowest.key))) {
                nextLowest = k
            }
        }

        return nextLowest

    }

}

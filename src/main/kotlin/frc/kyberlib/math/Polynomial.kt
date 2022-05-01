package frc.kyberlib.math

import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Representation and calculator of a polynomial
 */
class Polynomial(
    vararg val coeffs: Double,
    private val variableName: Char = 'x'
) {

    companion object {
        fun regress(args: DoubleArray, outputs: DoubleArray, order: Int =1) : Polynomial {
            var n = order
            val datasetSize = args.size
            val X = DoubleArray(2 * n + 1)
            for (i in 0 until 2 * n + 1) {
                X[i] = 0.0
                for (j in 0 until datasetSize) X[i] = X[i] + args[j].pow(i.toDouble()) //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
            }
            val B = Array(n + 1) { DoubleArray(n + 2) }
            val a = DoubleArray(n + 1) //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients

            for (i in 0..n) for (j in 0..n) B[i][j] =
                X[i + j] //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix

            val Y = DoubleArray(n + 1) //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)

            for (i in 0 until n + 1) {
                Y[i] = 0.0
                for (j in 0 until datasetSize) Y[i] = Y[i] + args[j].pow(i.toDouble()) * outputs[j] //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
            }
            for (i in 0..n) B[i][n + 1] = Y[i] //load the values of Y as the last column of B(Normal Matrix but augmented)

            n += 1
            for (i in 0 until n)  //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
                for (k in i + 1 until n) if (B[i][i] < B[k][i]) for (j in 0..n) {
                    val temp = B[i][j]
                    B[i][j] = B[k][j]
                    B[k][j] = temp
                }

            for (i in 0 until n - 1)  //loop to perform the gauss elimination
                for (k in i + 1 until n) {
                    val t = B[k][i] / B[i][i]
                    for (j in 0..n) B[k][j] =
                        B[k][j] - t * B[i][j] //make the elements below the pivot elements equal to zero or elimnate the variables
                }
            for (i in n - 1 downTo 0)  //back-substitution
            {                        //args is an array whose values correspond to the values of args,outputs,z..
                a[i] = B[i][n] //make the variable to be calculated equal to the rhs of the last equation
                for (j in 0 until n) if (j != i) //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                    a[i] = a[i] - B[i][j] * a[j]
                a[i] = a[i] / B[i][i] //now finally divide the rhs by the coefficient of the variable to be calculated
            }
            a.reverse()
            return Polynomial(*a)
        }
    }

    val degree = coeffs.size

    /**
     * Solve the polynomial for the given value
     */
    fun eval(x: Double): Double {
        var total = 0.0
        for (i in coeffs.indices) {
            total += coeffs[i] * x.pow(coeffs.size - i - 1)
        }
        return total
    }

    operator fun get(x: Double) = eval(x)

    fun r(data: DoubleArray, actualResults: DoubleArray): Double {
        val n = data.size
        return (n*(data.zip(actualResults).sumOf { it.first * it.second }) - data.sum() * actualResults.sum()) /
                sqrt(n*(data.sumOf { it*it }-data.sum())) / n*(actualResults.sumOf { it*it }-actualResults.sum())
    }

    override fun toString(): String {
        var s = ""
        for (i in coeffs.indices) {
            s += "${coeffs[i]}$variableName^${coeffs.size - i - 1}"
            if (i < coeffs.size - 1 && coeffs[i + 1] >= 0.0) s += "+"
        }
        return s
    }
}

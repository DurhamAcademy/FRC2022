package frc.kyberlib.motorcontrol.characterization

// https://www.bragitoff.com/2017/04/polynomial-fitting-java-codeprogram-works-android-well/
// todo: figure out how this works
class PolynomialFit(private val args: DoubleArray, private val outputs: DoubleArray, private var n: Int=1) {
    val N = args.size
    fun test() {
        val X = DoubleArray(2 * n + 1)
        for (i in 0 until 2 * n + 1) {
            X[i] = 0.0
            for (j in 0 until N) X[i] = X[i] + Math.pow(
                args.get(j),
                i.toDouble()
            ) //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
        }
        val B = Array(n + 1) { DoubleArray(n + 2) }
        val a = DoubleArray(n + 1) //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients

        for (i in 0..n) for (j in 0..n) B[i][j] =
            X[i + j] //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix

        val Y = DoubleArray(n + 1) //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)

        for (i in 0 until n + 1) {
            Y[i] = 0.0
            for (j in 0 until N) Y[i] = Y[i] + Math.pow(
                args.get(j),
                i.toDouble()
            ) * outputs.get(j) //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
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
    }

    override fun toString(): String {
        return super.toString()
    }
}
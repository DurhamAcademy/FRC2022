package frc.kyberlib.math.threeD
import frc.kyberlib.math.units.extensions.cos
import frc.kyberlib.math.units.extensions.radians
import frc.kyberlib.math.units.extensions.sin
import kotlin.math.*

/**
 * Represents quaternion. Allows for nicer interpolation of 3d angles
 *
 * @property w The first component.
 * @property x The second component.
 * @property y The third component.
 * @property z The fourth component.
 */
data class Quaternion(val w: Double=0.0, val x: Double=0.0, val y: Double=0.0, val z: Double=0.0) {
    private constructor(cy: Double, sy: Double, cp: Double, sp: Double, cr: Double, sr: Double) : this(cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)
    constructor(rotation3d: Rotation3d) : this ((rotation3d.yaw * 0.5).cos, (rotation3d.yaw * 0.5).sin, (rotation3d.pitch * 0.5).cos, (rotation3d.pitch * 0.5).sin, (rotation3d.roll * 0.5).cos, (rotation3d.roll * 0.5).sin)

    fun interpolate(other: Quaternion, alpha: Double=0.5) = this * (1-alpha) + other * alpha

    inline val rotation get() = Rotation3d(
        atan2(2*(w*x+y*z), 1-2*(x*x+y*y)).radians,
        asin(2*(w*y-z*x)).radians,
        atan2(2*(w*z+y*x), 1-2*(z*z+y*y)).radians
    )
    // operations
    operator fun plus(other: Quaternion) = Quaternion(w + other.w, x + other.x, y + other.y, z + other.z)
    operator fun minus(other: Quaternion) = Quaternion(w - other.w, x - other.x, y - other.y, z - other.z)
    operator fun times(other: Quaternion) = Quaternion(
        w * other.w - x * other.x - y * other.y - z * other.z,
        w * other.x + x * other.w + y * other.z - z * other.y,
        w * other.y - x * other.z + y * other.w + z * other.x,
        w * other.z + x * other.y - y * other.x + z * other.w,
    )
    operator fun div(other: Quaternion): Quaternion {
        val s = other.w * other.w + other.x * other.x + other.y * other.y + other.z * other.z

        return Quaternion(
            (other.w * w + other.x * x + other.y * y + other.z * z) / s,
            (other.w * x - other.x * w - other.y * z + other.z * y) / s,
            (other.w * y + other.x * z - other.y * w - other.z * x) / s,
            (other.w * z - other.x * y + other.y * x - other.z * w) / s,
        )
    }
    operator fun unaryMinus(): Quaternion = Quaternion(-w, -x, -y, -z)
    operator fun times(number: Double) = scale(number)
    operator fun div(number: Double) = scale(1/number)

    fun scale(value: Double): Quaternion = Quaternion(w * value, x * value, y * value, z * value)
    fun ln(): Quaternion {
        val nu2 = x * x + y * y + z * z

        if (nu2 == 0.0) Quaternion(ln(w), 0.0, 0.0, 0.0)

        val a = w
        check(nu2 > 0)
        val n = sqrt(a * a + nu2)
        val th = acos(a / n) / sqrt(nu2)
        return Quaternion(ln(n), th * x, th * y, th * z)
    }

    inline val sinh get() = (exp(this) - exp(-this)) / 2.0
    inline val cosh get() = (exp(this) + exp(-this)) / 2.0
    inline val tanh get() = (exp(this) - exp(-this)) / (exp(-this) + exp(this))
    inline val asinh get() = ln(sqrt(this * this + this) + this)
    inline val acosh get() = ln(this + sqrt((this - one) * (this + one)))
    inline val atanh get() = (ln(this) - ln(one - this)) / 2.0

    inline val r get() = sqrt(w * w + x * x + y * y + z * z)
    inline val absoluteValue get() = r
    inline val conjugate get() = Quaternion(z, -x, -y, -z)
    inline val reciprocal get() = this / r.pow(2)
    inline val norm get() = this / r
    companion object {
        val one = Quaternion(1.0)

        fun ln(arg: Quaternion) = arg.ln()
        fun power(arg: Quaternion, pow: Double): Quaternion = exp(ln(arg) * pow)

        fun exp(arg: Quaternion): Quaternion {
            val un = arg.x * arg.x + arg.y * arg.y + arg.z * arg.z
            if (un == 0.0) return Quaternion(exp(arg.w))
            val n1 = sqrt(un)
            val ea = exp(arg.w)
            val n2 = ea * sin(n1) / n1
            return Quaternion(ea * cos(n1), n2 * arg.x, n2 * arg.y, n2 * arg.z)
        }

        fun sqrt(arg: Quaternion) = power(arg, .5)
    }

    /**
     * Returns a string representation of this quaternion.
     */
    override fun toString(): String = "($w + $x e1 + $y e2 + $z k)"
}
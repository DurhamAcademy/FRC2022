package frc.kyberlib.math.units

import frc.kyberlib.math.Stopwatch
import frc.kyberlib.math.epsilonEquals
import frc.kyberlib.math.units.extensions.*
import kotlin.math.absoluteValue

/**
 * Unit system inspired by those of FalconLibrary and SaturnLibrary.
 * Allows for dimensional analysis
 * @author Trevor
 */
@JvmInline
value class KUnit<T>(val value: Double) : Comparable<KUnit<T>> {
    // math functions
    operator fun plus(other: KUnit<T>): KUnit<T> = KUnit(value + other.value)
    operator fun minus(other: KUnit<T>): KUnit<T> = KUnit(value - other.value)
    operator fun div(other: KUnit<T>) = this.value / other.value
    operator fun times(other: Double): KUnit<T> = KUnit<T>(value * other)
    operator fun div(other: Double): KUnit<T> = KUnit<T>(value / other)
    operator fun rem(other: KUnit<T>) = KUnit<T>(value % other.value)

    val absoluteValue inline get() = KUnit<T>(value.absoluteValue)
    override fun compareTo(other: KUnit<T>) = value.compareTo(other.value)

    /**
     * Check if other is very close to this
     */
    infix fun epsilonEquals(other: KUnit<T>) = value epsilonEquals other.value

    operator fun unaryMinus(): KUnit<T> = KUnit(-value)

    fun testing() {
//        T.unit
    }
}

// combining separate units
operator fun <T : KUnitKey, U : KUnitKey> KUnit<T>.times(other: KUnit<U>): KUnit<Mul<T, U>> =
    KUnit<Mul<T, U>>(value * other.value)

operator fun <T : KUnitKey, U : KUnitKey> KUnit<T>.div(other: KUnit<U>): KUnit<Div<T, U>> =
    KUnit<Div<T, U>>(value / other.value)

@JvmName("stringDiv")
inline fun <reified T : KUnitKey, reified U : KUnitKey> KUnit<Div<T, U>>.string(): String =
    "$value ${T::class.java.simpleName}s per ${U::class.java.simpleName}"

@JvmName("stringMul")
inline fun <reified T : KUnitKey, reified U : KUnitKey> KUnit<Mul<T, U>>.string(): String =
    "$value ${T::class.java.simpleName} ${U::class.java.simpleName}s"

inline fun <reified T : KUnitKey> KUnit<T>.string(): String = "$value ${T::class.java.simpleName}s"

@JvmName("unitsDiv")
inline fun <reified T : KUnitKey, reified U : KUnitKey> KUnit<Div<T, U>>.units(): String =
    "${T::class.java.simpleName}s per ${U::class.java.simpleName}"

@JvmName("unitsMul")
inline fun <reified T : KUnitKey, reified U : KUnitKey> KUnit<Mul<T, U>>.units(): String =
    "${T::class.java.simpleName} ${U::class.java.simpleName}s"

inline fun <reified T : KUnitKey> KUnit<T>.units(): String = "${T::class.java.simpleName}s"


fun main() {
    // conversions
//    println(45.degrees.string())
//    println(45.radiansPerSecond.units())
//    println((5.seconds * 10.seconds).units())
//    println(180.degrees.toCircumference(1.meters))
//    println(1.meters.toAngle(1.meters))
//    println(1.metersPerSecond.toAngularVelocity(1.meters))
//    println(1.radiansPerSecond)

    // speed
    println("\nspeed")
    val testCases = 1000000000
    val s = Stopwatch()
    s.loop()
    for(i in 0 until testCases) {
        accept(0.rpm)
    }
    println(s.loop())
    for(j in 0 until testCases) {
        accept(0.0)
    }
    println(s.loop())
    for(k in 0 until testCases) {
        accept(0.radians)
    }
    println(s.loop())
}

fun accept(x: Double) = x*x
fun accept(x: KUnit<*>) = x.value * x.value

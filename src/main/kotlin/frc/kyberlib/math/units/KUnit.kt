package frc.kyberlib.math.units

import frc.kyberlib.math.epsilonEquals
import frc.kyberlib.math.units.extensions.*
import kotlin.math.absoluteValue

/**
 * Unit system inspired by those of FalconLibrary and SaturnLibrary.
 * Allows for dimensional analysis
 * @author Trevor
 */
inline class KUnit<T>(val value: Double) : Comparable<KUnit<T>> {
    // math functions
    operator fun plus(other: KUnit<T>): KUnit<T> {
        val unit = KUnit<T>(value + other.value)
        shareUnits(unit)
        return unit
    }
    operator fun minus(other: KUnit<T>): KUnit<T> {
        val unit = KUnit<T>(value - other.value)
        shareUnits(unit)
        return unit
    }
    operator fun div(other: KUnit<T>) = this.value / other.value
    operator fun times(other: KUnit<T>) = this.value / other.value

    operator fun times(other: Double): KUnit<T> {
        val unit = KUnit<T>(value * other)
        shareUnits(unit)
        return unit
    }
    operator fun div(other: Double): KUnit<T> {
        val unit = KUnit<T>(value / other)
        shareUnits(unit)
        return unit
    }

    val absoluteValue get() = KUnit<T>(value.absoluteValue)
    override fun compareTo(other: KUnit<T>) = value.compareTo(other.value)

    /**
     * Check if other is very close to this
     */
    infix fun epsilonEquals(other: KUnit<T>) = value epsilonEquals other.value

    operator fun unaryMinus(): KUnit<T> = KUnit(-value)

    override fun toString(): String {
        return "($value $units)"
    }
}

// combining separate units
operator fun <T : KUnitKey, U : KUnitKey> KUnit<T>.times(other: KUnit<U>): KUnit<Mul<T, U>>  = KUnit<Mul<T, U>>(value * other.value)
operator fun <T : KUnitKey, U : KUnitKey> KUnit<T>.div(other: KUnit<U>): KUnit<Div<T, U>> = KUnit<Div<T, U>>(value / other.value)

object KUnitTests {
    @JvmStatic
    fun main(args: Array<String>) {
        println(180.degrees.toCircumference(1.meters))
        println(1.meters.toAngle(1.meters))
        println(1.metersPerSecond.toAngularVelocity(1.meters))
        println(1.radiansPerSecond)
    }
}

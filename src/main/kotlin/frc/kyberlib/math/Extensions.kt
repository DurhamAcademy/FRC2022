package frc.kyberlib.math

import frc.kyberlib.math.units.*
import java.math.BigDecimal
import java.math.RoundingMode
import kotlin.math.absoluteValue
import kotlin.math.roundToInt
import kotlin.random.Random

val Double.rtf: Double get() = (this * 10000).roundToInt() / 10000.0

// conditional inversion
fun Double.invertIf(condition: () -> Boolean) = this * if (condition()) -1.0 else 1.0

fun Int.invertIf(condition: () -> Boolean) = this * if (condition()) -1 else 1

infix fun Double.epsilonEquals(value: Double): Boolean {
    return (this - value).absoluteValue < 0.00001
}

fun Double.round(decimals: Int): Double {
    return BigDecimal(this).setScale(decimals, RoundingMode.HALF_EVEN).toDouble()
}

fun Double.zeroIf(condition: (it: Double) -> Boolean) = if (condition(this)) 0.0 else this 


val Int.even: Boolean
    get() = this.rem(2) == 0

val Int.odd: Boolean
    get() = !even

val Number.sign
    get() = this.toDouble() / this.toDouble().absoluteValue

fun Number.between(low: Number, high: Number) = this.toDouble() >= low.toDouble()  && this.toDouble() <= high.toDouble()

val randomizer = java.util.Random()


@JvmName("toDiv")
inline infix fun <reified T : KUnitKey, reified U : KUnitKey> String.to(value: KUnit<Mul<T, U>>): Pair<String, Number>  = "$this (${value.units()})" to value.value
@JvmName("toMul")
inline infix fun <reified T : KUnitKey, reified U : KUnitKey> String.to(value: KUnit<Div<T, U>>): Pair<String, Number>  = "$this (${value.units()})" to value.value
inline infix fun <reified T: KUnitKey> String.to(value: KUnit<T>): Pair<String, Number> = "$this (${value.units()})" to value.value

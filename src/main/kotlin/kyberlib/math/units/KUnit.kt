package kyberlib.math.units

import kyberlib.math.epsilonEquals
import kyberlib.math.units.extensions.*
import kotlin.math.absoluteValue

/**
 * Unit system inspired by those of FalconLibrary and SaturnLibrary.
 * Allows for dimensional analysis
 * @author Trevor
 */
class KUnit<T>(val value: Double) : Comparable<KUnit<T>> {
    var units = "KUnit"

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

    /**
     * Removes redundant units from unit name
     */
    internal fun cancelExtraUnits() {
        val seq = units.splitToSequence(' ').toList()
        if (seq.size == 1) return
        val topList = mutableListOf<String>()
        val bottomList = mutableListOf<String>()
        var top = true
        for (item in seq) {
            if (item == "*") top = true
            else if (item == "/") top = false
            else {
                if (top) {
                    if (item in bottomList) bottomList.remove(item)
                    else topList.add(item)
                } else {
                    if (item in topList) topList.remove(item)
                    else bottomList.add(item)
                }
            }
        }
        val string = StringBuilder()
        var doParenthesises = bottomList.isNotEmpty() && topList.size > 1
        if (topList.isNotEmpty()) {
            if (doParenthesises) string.append("(")
            topList.forEachIndexed { index, s ->
                if (index != 0) string.append(" ")
                string.append(s)
                if (index != topList.size -1) string.append(" *")
            }
            if (doParenthesises) string.append(")")
        } else string.append("1")
        if (bottomList.isNotEmpty()) {
            string.append(" / ")
            doParenthesises = bottomList.size > 1
            if (doParenthesises) string.append("(")
            bottomList.forEachIndexed { index, s ->
                if (index != 0) string.append(" ")
                string.append(s)
                if (index != topList.size -1) string.append(" *")
            }
            if (doParenthesises) string.append(")")
        }
        this.units = string.toString()
    }

    /**
     * Asserts shared units with another KUnit
     */
    private fun shareUnits(other: KUnit<T>) {
        other.units = units
    }

    override fun toString(): String {
        return "($value $units)"
    }
}

// combining separate units
operator fun <T : KUnitKey, U : KUnitKey> KUnit<T>.times(other: KUnit<U>): KUnit<Mul<T, U>> {
    val unit = KUnit<Mul<T, U>>(value * other.value)
    unit.units = "$units * ${other.units}"
    unit.cancelExtraUnits()
    return unit
}
operator fun <T : KUnitKey, U : KUnitKey> KUnit<T>.div(other: KUnit<U>): KUnit<Div<T, U>> {
    val unit = KUnit<Div<T, U>>(value / other.value)
    unit.units = "$units / ${other.units}"
    unit.cancelExtraUnits()
    return unit
}


object KUnitTests {
    @JvmStatic
    fun main(args: Array<String>) {
        println(180.degrees.toCircumference(1.meters))
        println(1.meters.toAngle(1.meters))
        println(1.metersPerSecond.toAngularVelocity(1.meters))
        println(1.radiansPerSecond)
    }
}

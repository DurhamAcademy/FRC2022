package frc.kyberlib.math.units.extensions

import edu.wpi.first.math.geometry.Rotation2d
import frc.kyberlib.math.units.*
import kotlin.math.PI

typealias Angle = KRotation

/**
 * KUnit for the rotation of an object.
 * Inherits from WpiLib's Rotation2d, so it can be used in all Wpi functions.
 * * This should not be initialized directly * - use Number.radians instead
 * @param value double value presenting the number of rotations.
 */
class KRotation(val value: Double) : Rotation2d(value), Comparable<Rotation2d> {
    val rotations: Double
        get() = value / AngleConversions.rotationsToRadians
    val normalized: Angle
        get() {
            val floored = ((rotations - rotations.toInt())).rotations
            return if (floored.value > 0) floored else (floored + 1.rotations).k
        }

    fun encoderAngle(cpr: Int) = (value / AngleConversions.rotationsToRadians) * (cpr * 4)
    fun toCircumference(radius: Length) = (radians * radius.meters).meters
    fun subtractNearest(other: Angle): Angle {
        val diff = (value - other.value + PI) % TAU - PI
        return Angle(if (diff < -PI) diff + TAU else diff)
    }

    override operator fun plus(other: Rotation2d): Angle = Angle(this.radians + other.radians)
    operator fun plus(other: Angle): Angle = Angle(this.radians + other.radians)
    override operator fun minus(other: Rotation2d): Angle = Angle(this.radians - other.radians)
    operator fun minus(other: Angle): Angle = Angle(this.radians - other.radians)

    override fun compareTo(other: Rotation2d) = this.radians.compareTo(other.radians)

    override fun toString(): String {
        return "($radians Radians)"
    }
}

operator fun Rotation2d.div(other: Double) = Angle(this.radians / other)
/**
 * Allows for division of angle by time.
 * Creates Angular Velocity
 */
operator fun Rotation2d.div(other: KUnit<Second>): KUnit<Div<Radian, Second>> {
    val unit = KUnit<Div<Radian, Second>>(radians / other.value)
    unit.units = "Radians / ${other.units}"
    return unit
}

/**
 * Convert WPI Rotation2d to kyberlib's KRotaion2d
 */
val Rotation2d.k: KRotation
    get() = KRotation(this.radians)

const val TAU = 2 * PI

// adds functions to all Number primitives
// ie 1.radians = KRotation2d(1.0)
val Number.radians get() = Angle(this.toDouble())
val Number.degrees get() = Angle(this.toDouble() * AngleConversions.degreesToRadians)
val Number.rotations get() = Angle(this.toDouble() * AngleConversions.rotationsToRadians)
fun Number.encoderAngle(cpr: Int) = (this.toDouble() / (cpr * 4.0)).rotations


fun main() {
    println(45.degrees == 405.degrees)
    println(45.degrees)
    println(405.degrees - 45.degrees)
    println((-370).degrees.normalized.degrees)
}
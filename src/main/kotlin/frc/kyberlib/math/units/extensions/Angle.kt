package frc.kyberlib.math.units.extensions

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import frc.kyberlib.math.units.*
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

typealias Angle = KUnit<Radian>

val Angle.rotations inline get() = value / AngleConversions.rotationsToRadians
val Angle.degrees inline get() = value / AngleConversions.degreesToRadians
val Angle.radians inline get() = value
val Angle.normalized inline get() = MathUtil.angleModulus(value).radians

val Angle.w inline get() = Rotation2d(value)

val Angle.sin inline get() = sin(value)
val Angle.cos inline get() = cos(value)
val Angle.sec inline get() = 1/cos
val Angle.csc inline get() = 1/sin
val Angle.tan inline get() = sin/cos
val Angle.cot inline get() = cos/sin

fun Angle.toCircumference(radius: Length) = (radians * radius.meters).meters
fun Angle.subtractNearest(other: Angle): Angle {
    val diff = (value - other.value + PI) % TAU - PI
    return Angle(if (diff < -PI) diff + TAU else diff)
}

/**
 * Allows for division of angle by time.
 * Creates Angular Velocity
 */
operator fun Angle.div(other: KUnit<Second>): KUnit<Div<Radian, Second>> = KUnit(radians / other.value)

/**
 * Convert WPI Rotation2d to kyberlib's KRotaion2d
 */
val Rotation2d.k: Angle inline get() = Angle(this.radians)
const val TAU = 2 * PI

// adds functions to all Number primitives
// ie 1.radians = KRotation2d(1.0)
val Number.radians inline get() = Angle(this.toDouble())
val Number.degrees inline get() = Angle(this.toDouble() * AngleConversions.degreesToRadians)
val Number.rotations inline get() = Angle(this.toDouble() * AngleConversions.rotationsToRadians)
fun Number.encoderAngle(cpr: Int) = (this.toDouble() / (cpr * 4.0)).rotations


fun main() {
    println(45.degrees == 405.degrees)
    println(45.degrees)
    println(405.degrees - 45.degrees)
    println((-370).degrees.normalized.degrees)
}
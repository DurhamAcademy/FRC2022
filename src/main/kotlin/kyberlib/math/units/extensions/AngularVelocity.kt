package kyberlib.math.units.extensions

import kyberlib.math.units.*

/**
 * KUnit representing angular velocity.
 * Should not be created directly. Use the number extensions instead.
 */
typealias AngularVelocity = KUnit<Div<Radian, Second>>

// creates Angular Velocity from Numbers
val Number.radiansPerSecond get() = this.radians / 1.seconds
val Number.degreesPerSecond get() = this.degrees / 1.seconds
val Number.rpm get() = this.rotations / 1.minutes
val Number.rotationsPerSecond get() = this.rotations / 1.seconds
fun Number.encoderVelocity(cpr: Int) = ((this.toDouble() / (cpr * 4.0)) * AngleConversions.rotationsToRadians * 10).radiansPerSecond

// converts Angular Velocities back to doubles
val AngularVelocity.radiansPerSecond get() = value
val AngularVelocity.degreesPerSecond get() = value / AngleConversions.degreesToRadians
val AngularVelocity.rpm get() = value * TimeConversions.minutesToSeconds / AngleConversions.rotationsToRadians
val AngularVelocity.rotationsPerSecond get() = value / AngleConversions.rotationsToRadians
fun AngularVelocity.toTangentialVelocity(radius: Length) = (value * radius.value).metersPerSecond
fun AngularVelocity.encoderVelocity(cpr: Int) = (value / (AngleConversions.rotationsToRadians * 10)) * (cpr * 4)
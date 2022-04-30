package frc.kyberlib.math.units.extensions

import frc.kyberlib.math.units.*

/**
 * KUnit representing angular angularVelocity.
 * Should not be created directly. Use the number extensions instead.
 */
typealias AngularVelocity = KUnit<Div<Radian, Second>>

// creates Angular Velocity from Numbers
val Number.radiansPerSecond inline get() = this.radians / 1.seconds
val Number.degreesPerSecond inline get() = this.degrees / 1.seconds
val Number.rpm inline get() = this.rotations / 1.minutes
val Number.rotationsPerSecond inline get() = this.rotations / 1.seconds
val Number.falconSpeed inline get() = (this.toDouble() / 2048.0).rotations / 100.milliseconds
fun Number.encoderVelocity(cpr: Int) =
    ((this.toDouble() / (cpr * 4.0)) * AngleConversions.rotationsToRadians * 10).radiansPerSecond

// converts Angular Velocities back to doubles
val AngularVelocity.radiansPerSecond inline get() = value
val AngularVelocity.degreesPerSecond inline get() = value / AngleConversions.degreesToRadians
val AngularVelocity.rpm inline get() = value * TimeConversions.minutesToSeconds / AngleConversions.rotationsToRadians
val AngularVelocity.rotationsPerSecond inline get() = value / AngleConversions.rotationsToRadians
val AngularVelocity.falconSpeed inline get() = rotationsPerSecond * 2048.0 / 10.0
fun AngularVelocity.toTangentialVelocity(radius: Length) = (value * radius.value).metersPerSecond
fun AngularVelocity.encoderVelocity(cpr: Int) = (value / (AngleConversions.rotationsToRadians * 10)) * (cpr * 4)

@JvmName("times")
operator fun AngularVelocity.times(radius: Length): LinearVelocity = this.toTangentialVelocity(radius)
operator fun AngularVelocity.times(time: Time): Angle = Angle(this.radiansPerSecond * time.seconds)
//operator fun Rotation2d.div(time: Time): AngularVelocity = AngularVelocity(this.radians * time.inWholeSeconds.toInt())
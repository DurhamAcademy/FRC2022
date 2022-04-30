package frc.kyberlib.math.units.extensions

import frc.kyberlib.math.units.*

/**
 * KUnit representing linear angularVelocity.
 * Should not be created directly. Use the number extensions instead.
 */
typealias LinearVelocity = KUnit<Div<Meter, Second>>

// number -> linearVelocity
val Number.metersPerSecond inline get() = this.meters / 1.seconds
val Number.millimetersPerSecond inline get() = this.millimeters / 1.seconds
val Number.feetPerSecond inline get() = this.feet / 1.seconds
val Number.milesPerHour inline get() = this.miles / 1.hours

// LinearVelocity -> double
val LinearVelocity.metersPerSecond inline get() = value
val LinearVelocity.millimetersPerSecond inline get() = value * 1000.0
val LinearVelocity.feetPerSecond inline get() = value / LengthConversions.feetToMeters
val LinearVelocity.milesPerHour inline get() = value / (LengthConversions.milesToFeet * LengthConversions.feetToMeters / (TimeConversions.hoursToMinutes * TimeConversions.minutesToSeconds))
fun LinearVelocity.toAngularVelocity(radius: Length) = AngularVelocity(value / radius.value)

operator fun LinearVelocity.div(radius: Length): AngularVelocity = this.toAngularVelocity(radius)
operator fun LinearVelocity.times(time: Time): Length = Length(this.metersPerSecond * time.seconds)
@JvmName("div")
operator fun Length.div(time: Time): LinearVelocity = LinearVelocity(this.meters / time.seconds)

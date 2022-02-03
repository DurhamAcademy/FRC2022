package kyberlib.math.units.extensions

import kyberlib.math.units.*

/**
 * KUnit representing linear velocity.
 * Should not be created directly. Use the number extensions instead.
 */
typealias LinearVelocity = KUnit<Div<Meter, Second>>

// number -> linearVelocity
val Number.metersPerSecond get() = this.meters / 1.seconds
val Number.feetPerSecond get() = this.feet / 1.seconds
val Number.milesPerHour get() = this.miles / 1.hours

// LinearVelocity -> double
val LinearVelocity.metersPerSecond get() = value
val LinearVelocity.feetPerSecond get() = value / LengthConversions.feetToMeters
val LinearVelocity.milesPerHour get() = value / (LengthConversions.milesToFeet * LengthConversions.feetToMeters / (TimeConversions.hoursToMinutes * TimeConversions.minutesToSeconds))
fun LinearVelocity.toAngularVelocity(radius: Length) = (value / radius.value).radiansPerSecond
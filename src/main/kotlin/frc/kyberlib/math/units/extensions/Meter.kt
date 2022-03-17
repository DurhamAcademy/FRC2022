package frc.kyberlib.math.units.extensions

import frc.kyberlib.math.units.KUnit
import frc.kyberlib.math.units.LengthConversions.feetToMeters
import frc.kyberlib.math.units.LengthConversions.inchesToFeet
import frc.kyberlib.math.units.LengthConversions.milesToFeet
import frc.kyberlib.math.units.Meter
import frc.kyberlib.math.units.centi


/**
 * KUnit representing length.
 * Should not be created directly. Use the number extensions instead.
 */
typealias Length = KUnit<Meter>

// Number -> length
val Number.meters inline get() = Length(this.toDouble())
val Number.centimeters inline get() = Length(this.centi)
val Number.millimeters inline get() = Length(this.toDouble() / 1000.0)
val Number.miles inline get() = Length(this.toDouble() * milesToFeet * feetToMeters)
val Number.feet inline get() = Length(this.toDouble() * feetToMeters)
val Number.inches inline get() = Length(this.toDouble() * inchesToFeet * feetToMeters)

// length -> Number
val Length.meters inline get() = value
val Length.centimeters inline get() = value / 1.centi
val Length.millimeters inline get() = value * 1000.0
val Length.miles inline get() = value / feetToMeters / milesToFeet
val Length.feet inline get() = value / feetToMeters
val Length.inches inline get() = value / (inchesToFeet * feetToMeters)
fun Length.toAngle(radius: Length) = (value / radius.value).radians

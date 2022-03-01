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

internal fun Distance(value: Double): Length {
    val unit = Length(value)
    unit.units = "Meters"
    return unit
}

// Number -> length
val Number.meters get() = Distance(this.toDouble())
val Number.centimeters get() = Distance(this.centi)
val Number.miles get() = Distance(this.toDouble() * milesToFeet * feetToMeters)
val Number.feet get() = Distance(this.toDouble() * feetToMeters)
val Number.inches get() = Distance(this.toDouble() * inchesToFeet * feetToMeters)

// length -> Number
val Length.meters get() = value
val Length.centimeters get() = value / 1.centi
val Length.miles get() = value / feetToMeters / milesToFeet
val Length.feet get() = value / feetToMeters
val Length.inches get() = value / (inchesToFeet * feetToMeters)
fun Length.toAngle(radius: Length) = (value / radius.value).radians

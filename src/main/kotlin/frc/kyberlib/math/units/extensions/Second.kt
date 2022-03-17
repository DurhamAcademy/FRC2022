package frc.kyberlib.math.units.extensions

import frc.kyberlib.math.units.KUnit
import frc.kyberlib.math.units.Second
import frc.kyberlib.math.units.TimeConversions
import frc.kyberlib.math.units.milli

/**
 * KUnit representing time.
 * Should not be created directly. Use the number extensions instead.
 */
typealias Time = KUnit<Second>

// Number to Time
val Number.milliseconds inline get() = this.milli.seconds
val Number.seconds inline get() = Time(this.toDouble())
val Number.minutes inline get() = Time(this.toDouble() * TimeConversions.minutesToSeconds)
val Number.hours inline get() = Time(this.toDouble() * TimeConversions.minutesToSeconds)

// conversions from Time back to double values
val Time.milliseconds inline get() = value * 1000.0
val Time.seconds inline get() = value
val Time.minutes inline get() = value / TimeConversions.minutesToSeconds
val Time.hours inline get() = value / (TimeConversions.hoursToMinutes * TimeConversions.minutesToSeconds)
val Time.string: String
    get() = "hi"
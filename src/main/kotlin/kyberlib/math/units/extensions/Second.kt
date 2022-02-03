package kyberlib.math.units.extensions

import kyberlib.math.units.KUnit
import kyberlib.math.units.Second
import kyberlib.math.units.TimeConversions

/**
 * KUnit representing time.
 * Should not be created directly. Use the number extensions instead.
 */
typealias Time = KUnit<Second>

/**
 * Creates a Time unit and sets the unit to seconds.
 * Only called from internal
 */
internal fun time(value: Double): Time {
    val unit = Time(value)
    unit.units = "Seconds"
    return unit
}

// Number to Time
val Number.seconds get() = time(this.toDouble())
val Number.minutes get() = time(this.toDouble() * TimeConversions.minutesToSeconds)
val Number.hours get() = time(this.toDouble() * TimeConversions.minutesToSeconds)

// conversions from Time back to double values
val Time.seconds get() = value
val Time.minutes get() = value / TimeConversions.minutesToSeconds
val Time.hours get() = value / (TimeConversions.hoursToMinutes * TimeConversions.minutesToSeconds)
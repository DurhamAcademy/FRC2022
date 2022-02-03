package kyberlib.math.units

/**
 * Represents a unit type
 */
open class KUnitKey {}

// combinations of units (doesn't work beyond one change - see testing for potential workarounds)
class Mul<T : KUnitKey, U : KUnitKey> : KUnitKey() {}
class Div<T : KUnitKey, U : KUnitKey> : KUnitKey()

// static object representing unit type
object Unitless : KUnitKey()
object Radian : KUnitKey()  // base unit for angle
object Meter : KUnitKey()  // base unit for length
object Second : KUnitKey()  // base unit for time
object Kelvin : KUnitKey()  // base unit for temp
object Mole : KUnitKey()  // base unit for amount
object Amp : KUnitKey()  // base unit for current
object Candela : KUnitKey()  // base unit for luminosity
object Kilogram : KUnitKey()  // base unit for mass

val unitKeyMap = mapOf(
    Unitless::class.simpleName to Unitless,
    Radian::class.simpleName to Radian,
    Meter::class.simpleName to Meter,
    Second::class.simpleName to Second,
    Kelvin::class.simpleName to Kelvin,
    Mole::class.simpleName to Mole,
    Amp::class.simpleName to Amp,
    Candela::class.simpleName to Candela,
    Kilogram::class.simpleName to Kilogram
)

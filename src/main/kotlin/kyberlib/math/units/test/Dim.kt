package kyberlib.math.units.test

/**
 * Tate was messing around trying to have completely dynamic SI unit math.
 * Doesn't really work without creating a mess
 */
open class Dim {
    // the power of each unit
    open val angle = 0
    open val length = 0
    open val time = 0
    open val temperature = 0
    open val amount = 0
    open val current = 0
    open val luminosity = 0
    open val mass = 0

    private val powerMap
        get() = mapOf(
            "rad" to angle,
            "m" to length,
            "sec" to time,
            "K" to temperature,
            "amp" to current,
            "cd" to luminosity,
            "kg" to mass
        )

    override fun toString(): String {
        val string = StringBuilder()
        powerMap.forEach { unit, power -> if (power > 1) string.append("$unit^$power")}
        return string.toString()
    }

    operator fun times(other: Dim): Dim {
        val caller = this
        return object : Dim() {
            override val angle = caller.angle + other.angle
            override val length = caller.length + other.length
            override val time = caller.time + other.time
            override val temperature = caller.temperature + other.temperature
            override val amount = caller.amount + other.amount
            override val current = caller.current + other.current
            override val luminosity = caller.luminosity + other.luminosity
            override val mass = caller.mass + other.mass
        }
    }

    operator fun div(other: Dim): Dim {
        val caller = this
        return object : Dim() {
            override val angle = caller.angle - other.angle
            override val length = caller.length - other.length
            override val time = caller.time - other.time
            override val temperature = caller.temperature - other.temperature
            override val amount = caller.amount - other.amount
            override val current = caller.current - other.current
            override val luminosity = caller.luminosity - other.luminosity
            override val mass = caller.mass - other.mass
        }
    }
}

object Unitless: Dim()  // unitless Unit
object Radian : Dim() { override val angle = 1 }  // base unit for angle
object Meter : Dim() { override val length = 1 }   // base unit for length
object Second : Dim() { override val time = 1 }   // base unit for time
object Kelvin : Dim() { override val temperature = 1 }   // base unit for temp
object Mole : Dim() { override val amount = 1 }   // base unit for amount
object Amp : Dim() { override val current = 1 }   // base unit for current
object Candela : Dim() { override val luminosity = 1 }   // base unit for luminosity
object Kilogram : Dim() { override val mass = 1 }   // base unit for mass

val unitKeyMap = mapOf(
    Radian::class.simpleName to Radian,
    Meter::class.simpleName to Meter,
    Second::class.simpleName to Second,
    Kelvin::class.simpleName to Kelvin,
    Mole::class.simpleName to Mole,
    Amp::class.simpleName to Amp,
    Candela::class.simpleName to Candela,
    Kilogram::class.simpleName to Kilogram
)
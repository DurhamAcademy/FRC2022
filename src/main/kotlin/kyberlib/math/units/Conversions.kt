package kyberlib.math.units

import kotlin.math.PI

/**
 * This file provides the unit conversions for different descriptors of the 7 SI units
 */

object LengthConversions {
    const val feetToMeters = 0.3048
    const val inchesToFeet = 1.0 / 12
    const val milesToFeet = 5280
}

object AngleConversions {
    const val degreesToRadians = PI / 180.0
    const val rotationsToRadians = 2 * PI
}

object TimeConversions {
    const val minutesToSeconds = 60
    const val hoursToMinutes = 60
}

// temperature (kelvin)
// these are multipliers so this is left blanks

// amount of substance (moles)
object AmountConversions {
    const val quantityToMoles = 6.022e23
}

// electric current (amp)
// there are no other units for Amps

// luminosity (candela)

// mass (kg)
object MassConversions {
    const val ouncesToPounds = 16.0
    const val poundsToGrams = 453.592
}


object Prefixes {
    const val deci = 1.0e-1         //	d	10^-1	0.1	tenth
    const val centi = 1.0e-2        //	c	10^-2	0.01	hundredth
    const val milli = 1.0e-3        //	m	10^-3	0.001	thousandth
    const val micro = 1.0e-6        //	µ	10^-6	0.000 001	millionth
    const val  nano = 1.0e-9        //	n	10^-9	0.000 000 001	billionth
    const val pico = 1.0e-12        //	p	10^-12	0.000 000 000 001	trillionth
    const val femto = 1.0e-15       //	f	10^-15	0.000 000 000 000 001	quadrillionth
    const val atto = 1.0e-18        //	a	10^-18	0.000 000 000 000 000 001	quintillionth
    const val zepto = 1.0e-21       //	z	10^-21	0.000 000 000 000 000 000 001	sextillionth
    const val yocto = 1.0e-24       //	y	10^-24	0.000 000 000 000 000 000 000 001	septillionth

    const val deca = 1.0e-1     	//  da	10^1	10	ten
    const val hecto = 1.0e-2     	//  h	10^2	100	hundred
    const val kilo = 1.0e-3     	//  k	10^3	1,000	thousand
    const val mega = 1.0e-6     	//  M	10^6	1,000,000	million
    const val giga = 1.0e-9     	//  G	10^9	1,000,000,000	billion
    const val tera = 1.0e-12     	//  T	10^12	1,000,000,000,000	trillion
    const val peta = 1.0e-15     	//  P	10^15	1,000,000,000,000,000	quadrillion
    const val exa = 1.0e-18         //  E	10^18	1,000,000,000,000,000,000	quintillion
    const val zetta = 1.0e-21     	//  Z	10^21	1,000,000,000,000,000,000,000	sextillion
    const val yotta = 1.0e-24     	//  Y	10^24	1,000,000,000,000,000,000,000,000	septillion
}

package kyberlib.math.units.test

/*
 * Copyright 2019 Kunal Sheth
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

import java.io.File
import java.io.Serializable

/**
 * Created by kunal on 8/6/17.
 */
@Suppress("UNCHECKED_CAST")
interface Quan<This> : Comparable<This> where This : Quan<This> {
    val siValue: Double
    val abrev: String

    fun new(siValue: Double): This

    operator fun unaryPlus(): This
    operator fun unaryMinus(): This

    operator fun plus(that: This): This
    operator fun minus(that: This): This
    operator fun times(that: Number): This
    operator fun div(that: Number): This
    operator fun rem(that: This): This

    infix fun min(that: This): This
    infix fun max(that: This): This

    val abs: This
    val signum: Double
    val isNegative: Boolean
    val isZero: Boolean
    val isPositive: Boolean

    override fun compareTo(other: This): Int
}

sealed class OperationProof
object times : OperationProof()
object div : OperationProof()

interface UomConverter<Q : Quan<Q>> {
    val unitName: String
    operator fun invoke(x: Double): Q
    operator fun invoke(x: Q): Double
}

//@Deprecated(
//    message = "Explicit boxing should only be used to circumnavigate compiler bugs",
//    replaceWith = ReplaceWith("a newer version of Kotlin")
//)
//fun <Q : Quan<Q>> box(x: Q) = x as Quan<Q>


data class Dimension(val name: String="Unitless", val L: Int = 0, val A: Int = 0, val M: Int = 0, val T: Int = 0, val I: Int = 0, val Theta: Int = 0, val N: Int = 0, val J: Int = 0) : Serializable {

    init {
        if (name != "Unitless") {
            InternationalSystemOfUnits.quantities.add(Quantity(name, this))
            InternationalSystemOfUnits.units.add(UnitOfMeasure(unitType[name]!!, 1.0, this))
        }
    }
    val safeName by lazy {
        val (numerator, denominator) = mapOf(
            "L" to L,
            "A" to A,
            "M" to M,
            "T" to T,
            "I" to I,
            "Theta" to Theta,
            "N" to N,
            "J" to J
        )
            .toList()
            .partition { (_, power) -> power >= 0 }

        val numeratorString = numerator
            .joinToString(separator = "") { (unit, power) -> unit + power }

        val denominatorString = denominator
            .map { (unit, power) -> unit to Math.abs(power) }
            .joinToString(separator = "") { (unit, power) -> unit + power }

        (numeratorString + if (denominatorString.isNotEmpty()) "_per_$denominatorString" else "")
            .takeUnless { it.isBlank() } ?: "Dimensionless"
    }

    val fancyName by lazy {
        mapOf(
            "L" to L,
            "∠" to A,
            "M" to M,
            "T" to T,
            "I" to I,
            "Θ" to Theta,
            "N" to N,
            "J" to J
        ).factorizedString
            .takeUnless(String::isBlank)
            ?: "Dimensionless"
    }

    val abbreviation by lazy {
        mapOf(
            "m" to L,
            "rad" to A,
            "kg" to M,
            "s" to T,
            "A" to I,
            "K" to Theta,
            "mol" to N,
            "cd" to J
        ).factorizedString
    }

    private val Map<String, Int>.factorizedString
        get() = filterValues { it != 0 }
            .mapValues { (_, power) ->
                power.toString()
                    .map {
                        when (it) {
                            '+' -> '⁺'
                            '-' -> '⁻'
                            '1' -> '¹'
                            '2' -> '²'
                            '3' -> '³'
                            '4' -> '⁴'
                            '5' -> '⁵'
                            '6' -> '⁶'
                            '7' -> '⁷'
                            '8' -> '⁸'
                            '9' -> '⁹'
                            else -> it
                        }
                    }
                    .joinToString(separator = "")
                    .takeUnless { it == "¹" } ?: ""
            }
            .map { (base, power) -> base + power }
            .joinToString(separator = "⋅")

    override fun toString() = "`$fancyName`"

    operator fun div(other: Dimension): Dimension {
        return Dimension(L = L - other.L, A = A - other.A, M = M - other.M, T = T - other.T, I = I - other.I, Theta = Theta - other.Theta, N = N - other.N, J = J - other.J)
    }
    operator fun times(other: Dimension): Dimension {
        return Dimension(L = L + other.L, A = A + other.A, M = M + other.M, T = T + other.T, I = I + other.I, Theta = Theta + other.Theta, N = N + other.N, J = J + other.J)
    }
}

private const val underlying = "underlying"
private const val siValue = "siValue"
private const val timesOperationProof = "times"
private const val divOperationProof = "div"
private fun quan(d: Dimension) = "Quan<$d>"

fun Dimension.src(relations: Set<Relation>, quantities: Set<Quantity>, units: Set<UnitOfMeasure>): String {
    return """
typealias $this = $safeName
class $safeName(internal val $underlying: Double) : ${quan(this)} {
    override val $siValue get() = $underlying
    override val abrev get() = "$abbreviation"

    override fun new($siValue: Double) = $this($siValue)

    override operator fun unaryPlus() = $this(+$underlying)
    override operator fun unaryMinus() = $this(-$underlying)

    override operator fun plus(that: $this) = $this(this.$underlying + that.$underlying)
    override operator fun minus(that: $this) = $this(this.$underlying - that.$underlying)
    override operator fun times(that: Number) = $this(this.$underlying * that.toDouble())
    override operator fun div(that: Number) = $this(this.$underlying / that.toDouble())
    override operator fun rem(that: $this) = $this(this.$underlying % that.$underlying)

    override infix fun min(that: $this) = if (this < that) this else that
    override infix fun max(that: $this) = if (this > that) this else that

    override val abs get() = $this(abs($underlying))
    override val signum get() = $underlying.sign
    override val isNegative get() = $underlying < 0
    override val isZero get() = $underlying == 0.0
    override val isPositive get() = $underlying > 0

    override fun compareTo(other: $this) = this.$underlying.compareTo(other.$underlying)

    override fun toString() = "${'$'}$underlying ${'$'}abrev"
    // override fun equals(other: Any?) = other is $this && this.siValue == other.siValue
}
${units.joinToString(separator = "") {
        it.src(quantities
            .takeIf { it.size == 1 }
            ?.run(Set<Quantity>::first)
        )
    }}
${quantities.joinToString(separator = "", transform = Quantity::src)}
${relations.joinToString(separator = "\n", transform = Relation::src)}
"""
}

private fun Relation.src(): String {
    fun jvmName(category: String) = """@JvmName("${a.safeName}_${f.name}_${b.safeName}_$category")"""

    return when (f) {
        RelationType.Divide -> {
            val logic = "$result(this.$siValue / that.$siValue)"
            """
            ${jvmName("generic")}
            operator fun $a.div(that: ${quan(b)}) = $logic
            // ${jvmName("concrete")}
            operator fun $a.div(that: $b) = $logic
            ${jvmName("proof")}
            fun p(thiz: ${quan(a)}, op: $divOperationProof, that: ${quan(b)}) = thiz.run { $logic }
        """.trimIndent()
        }
        RelationType.Multiply -> {
            val logic = "$result(this.$siValue * that.$siValue)"
            """
            ${jvmName("generic")}
            operator fun $a.times(that: ${quan(b)}) = $logic
            // ${jvmName("concrete")}
            operator fun $a.times(that: $b) = $logic
            ${jvmName("proof")}
            fun p(this: ${quan(a)}, op: $timesOperationProof, that: ${quan(b)}) = this.run { $logic }
        """.trimIndent()
        }
    }
}

private fun Quantity.src() = """
typealias $this = $dimension
"""

private fun UnitOfMeasure.src(quantity: Quantity?) = """
val Number.$this: ${quantity ?: dimension} get() = $dimension(toDouble() * $factorToSI)
val $dimension.$this get() = $siValue * ${1 / factorToSI}
object $this : UomConverter<$dimension>,
    ${quan(dimension)} by $dimension($factorToSI) {
    override val unitName = "$name"
    override fun invoke(x: Double) = x.$this
    override fun invoke(x: $dimension) = x.$this
}
"""

val mathDependencies = setOf(
    Dimension("Unitless"),
    Dimension("Length", L = 1),
    Dimension("Angle", A = 1),

    Dimension(L = 1, A = 1),
    Dimension(L = 1, T = -1, A = 1),
    Dimension(L = 1, T = -2, A = 1)
)


data class Quantity(val name: String, val dimension: Dimension) : Serializable {
    override fun toString() = "`$name`"
}

data class Relation(val a: Dimension, val f: RelationType, val b: Dimension) : Serializable {
    val result = f(a, b)

    companion object {
        private fun Set<Dimension>.permuteRelations() =
            flatMap { x ->
                flatMap { y ->
                    listOf(
                        Relation(x, RelationType.Divide, x),
                        Relation(x, RelationType.Divide, y),
                        Relation(x, RelationType.Multiply, x),
                        Relation(x, RelationType.Multiply, y),

                        Relation(y, RelationType.Divide, x),
                        Relation(y, RelationType.Divide, y),
                        Relation(y, RelationType.Multiply, x),
                        Relation(y, RelationType.Multiply, y)
                    )
                }
            }

        fun closedPermute(inputDimensions: Set<Dimension>) = inputDimensions.permuteRelations()
            .filter { arrayOf(it.a, it.b, it.result).all { it in inputDimensions } }
            .toSet()

        fun openPermute(inputDimensions: Set<Dimension>): Set<Relation> {
            val newDimensions = inputDimensions +
                    inputDimensions.permuteRelations().map(Relation::result)

            return newDimensions
                .permuteRelations()
                .filter { arrayOf(it.a, it.b, it.result).all { it in newDimensions } }
                .toSet()
        }
    }
}

enum class RelationType : (Dimension, Dimension) -> Dimension {
    Divide { override fun invoke(a: Dimension, b: Dimension) = a / b },
    Multiply { override fun invoke(a: Dimension, b: Dimension) = a * b }
}

data class UnitOfMeasure(val name: String, val factorToSI: Double, val dimension: Dimension) : Serializable {
    override fun toString() = "`$name`"
}

object InternationalSystemOfUnits {
    @JvmStatic
    fun main(args: Array<String>) {
        generate()
    }
    fun generate() {
        val srcWriter = File("src/main/kotlin/kyberlib/math/units/test/SI.kt")
//        writeBase(srcWriter)

        val allRelations = Relation.closedPermute(mathDependencies +
                quantities.map(Quantity::dimension) +
                units.map(UnitOfMeasure::dimension)
        )

        val allDimensions = (mathDependencies +
                allRelations.flatMap { listOf(it.a, it.b, it.result) } +
                quantities.map(Quantity::dimension) +
                units.map(UnitOfMeasure::dimension))
            .toSet()

        val relationGroups = allRelations
            .groupBy { it.a }
            .mapValues { (_, v) -> v.toSet() }

        val quantityGroups = quantities
            .groupBy { it.dimension }
            .mapValues { (_, v) -> v.toSet() }

        val unitGroups = units
            .groupBy { it.dimension }
            .mapValues { (_, v) -> v.toSet() }

        val generatedSourceCode = allDimensions.map {
            it.src(
                relationGroups[it] ?: emptySet(),
                quantityGroups[it] ?: emptySet(),
                unitGroups[it] ?: emptySet()
            )
        }

        srcWriter.bufferedWriter().use { writer ->
            generatedSourceCode.forEach { writer.appendLine(it) }
            srcWriter.writeText(mathString)
        }
    }

    private val Length = Dimension("Length", L = 1)
    private val Mass = Dimension("Mass", M = 1)
    private val Time = Dimension("Time", T = 1)
    private val ElectricCurrent = Dimension("ElectricCurrent", I = 1)
    private val Temperature = Dimension("Temperature", Theta = 1)
    private val AmountOfSubstance = Dimension("AmountOfSubstance", N = 1)
    private val LuminousIntensity = Dimension("LuminousIntensity", J = 1)

    private val Frequency = Dimension("Frequency", T = -1)
    private val Angle = Dimension("Angle", A = 1)
//    private val SolidAngle = Dimension("SolidAngle", A = 2)
    private val Force = Dimension("Force", M = 1, L = 1, T = -2)
    private val Pressure = Dimension("Pressure", M = 1, L = -1, T = -2)
    private val Energy = Dimension("Energy", M = 1, L = 2, T = -2)
    private val Heat = Dimension("Heat", M = 1, L = 2, T = -2)
    private val Power = Dimension("Power", M = 1, L = 2, T = -3)
//    private val RadiantFlux = Dimension("RadiantFlux", M = 1, L = 2, T = -3)
//    private val ElectricCharge = Dimension("ElectricCharge", T = 1, I = 1)
    private val ElectricalPotential = Dimension("ElectricalPotential", M = 1, L = 2, T = -3, I = -1)
    private val ElectricalCapacitance = Dimension("ElectricalCapacitance", M = -1, L = -2, T = 4, I = 2)
    private val ElectricalResistance = Dimension("ElectricalResistance", M = 1, L = 2, T = -3, I = -2)
    private val ElectricalConductance = Dimension("ElectricalConductance", M = -1, L = -2, T = 3, I = 2)
//    private val MagneticFlux = Dimension("MagneticFlux", M = 1, L = 2, T = -2, I = -1)
//    private val MagneticFieldStrength = Dimension("MagneticFieldStrength", M = 1, T = -2, I = -1)
//    private val MagneticFluxDensity = Dimension("MagneticFluxDensity", M = 1, T = -2, I = -1)
    private val ElectricalInductance = Dimension("ElectricalInductance", M = 1, L = 2, T = -2, I = -2)
//    private val LuminousFlux = Dimension("LuminousFlux", A = 2, J = 1)
//    private val Illuminance = Dimension("Illuminance", L = -2, J = 1)
//    private val AbsorbedDose = Dimension("AbsorbedDose", L = 2, T = -2)
//    private val EquivalentDose = Dimension("EquivalentDose", L = 2, T = -2)
//    private val CatalyticActivity = Dimension("CatalyticActivity", T = -1, N = 1)

    private val Area = Dimension("Area", L = 2)
    private val Volume = Dimension("Volume", L = 3)
    private val Velocity = Dimension("Velocity", L = 1, T = -1)
//    private val VolumetricFlow = Dimension("VolumetricFlow", L = 3, T = -1)
    private val Acceleration = Dimension("Acceleration", L = 1, T = -2)
//    private val Jerk = Dimension(L = 1, T = -3)
//    private val Jolt = Dimension(L = 1, T = -3)
//    private val Snap = Dimension(L = 1, T = -4)
//    private val Jounce = Dimension(L = 1, T = -4)
//    private val AngularVelocity = Dimension(A = 1, T = -1)
//    private val AngularAcceleration = Dimension(A = 1, T = -2)
//    private val Momentum = Dimension(L = 1, M = 1, T = -1)
//    private val Impulse = Dimension(L = 1, M = 1, T = -1)
//    private val AngularMomentum = Dimension(L = 2, M = 1, T = -1)
//    private val Torque = Dimension(A = -1, L = 2, M = 1, T = -2)
    /**
    private val MomentOfForce = Dimension(L = 2, M = 1, T = -2)
    private val Yank = Dimension(L = 1, M = 1, T = -3)
    private val Wavenumber = Dimension(L = -1)
    private val OpticalPower = Dimension(L = -1)
    private val Curvature = Dimension(L = -1)
    private val SpatialFrequency = Dimension(L = -1)
    private val AreaDensity = Dimension(L = -2, M = 1)
    private val Density = Dimension(L = -3, M = 1)
    private val MassDensity = Dimension(L = -3, M = 1)
    private val SpecificVolume = Dimension(L = 3, M = -1)
    private val Molarity = Dimension(L = -3, N = 1)
    private val AmountOfSubstanceConcentration = Dimension(L = -3, N = 1)
    private val MolarVolume = Dimension(L = 3, N = -1)
    private val Action = Dimension(L = 2, M = 1, T = -1)
    private val HeatCapacity = Dimension(L = 2, M = 1, T = -2, Theta = -1)
    private val Entropy = Dimension(L = 2, M = 1, T = -2, Theta = -1)
    private val MolarHeatCapacity = Dimension(L = 2, M = 1, T = -2, Theta = -1, N = -1)
    private val MolarEntropy = Dimension(L = 2, M = 1, T = -2, Theta = -1, N = -1)
    private val SpecificHeatCapacity = Dimension(L = 2, T = -2, Theta = -1)
    private val SpecificEntropy = Dimension(L = 2, T = -2, Theta = -1)
    private val MolarEnergy = Dimension(L = 2, M = 1, T = -2, N = -1)
    private val SpecificEnergy = Dimension(L = 2, T = -2)
    private val EnergyDensity = Dimension(L = -1, M = 1, T = -2)
    private val SurfaceTension = Dimension(M = 1, T = -2)
    private val Stiffness = Dimension(M = 1, T = -2)
    private val HeatFluxDensity = Dimension(M = 1, T = -3)
    private val Irradiance = Dimension(M = 1, T = -3)
    private val ThermalConductivity = Dimension(L = 1, M = 1, T = -3, Theta = -1)
    private val KinematicViscosity = Dimension(L = 2, T = -1)
    private val ThermalDiffusivity = Dimension(L = 2, T = -1)
    private val DiffusionCoefficient = Dimension(L = 2, T = -1)
    private val DynamicViscosity = Dimension(L = -1, M = 1, T = -1)
    private val ElectricDisplacementField = Dimension(L = -2, T = 1, I = 1)
    private val PolarizationDensity = Dimension(L = -2, T = 1, I = 1)
    private val ElectricChargeDensity = Dimension(L = -3, T = 1, I = 1)
    private val ElectricCurrentDensity = Dimension(L = -2, I = 1)
    private val ElectricalConductivity = Dimension(L = -3, M = -1, T = 3, I = 2)
    private val MolarConductivity = Dimension(M = -1, T = 3, I = 2, N = -1)
    private val Permittivity = Dimension(L = -3, M = -1, T = 4, I = 2)
    private val MagneticPermeability = Dimension(L = 1, M = 1, T = -2, I = -2)
    private val ElectricFieldStrength = Dimension(L = 1, M = 1, T = -3, I = -1)
    private val Magnetization = Dimension(L = -1, I = 1)
    private val Luminance = Dimension(L = -2, J = 1)
    private val LuminousEnergy = Dimension(T = 1, J = 1)
    private val LuminousExposure = Dimension(L = -2, T = 1, J = 1)
    private val Exposure = Dimension(M = -1, T = 1, I = 1)
    private val AbsorbedDoseRate = Dimension(L = 2, T = -3)
    private val Resistivity = Dimension(L = 3, M = 1, T = -3, I = -2)
    private val LinearMassDensity = Dimension(L = -1, M = 1)
    private val LinearChargeDensity = Dimension(L = -1, T = 1, I = 1)
    private val Molality = Dimension(M = -1, N = 1)
    private val MolarMass = Dimension(M = 1, N = -1)
    private val FuelEfficiency = Dimension(L = -2)
    private val MassFlowRate = Dimension(M = 1, T = -1)
    private val MagneticDipoleMoment = Dimension(L = 2, I = 1)
    private val SpectralIrradiance = Dimension(L = -1, M = 1, T = -3)
    private val PowerDensity = Dimension(L = -1, M = 1, T = -3)
    private val ThermalResistance = Dimension(L = -2, M = -1, T = 3, Theta = 1)
    private val ThermalExpansionCoefficient = Dimension(Theta = -1)
    private val TemperatureGradient = Dimension(L = -1, Theta = 1)
    private val ElectronMobility = Dimension(M = -1, T = 2, I = 1)
    private val EnergyFluxDensity = Dimension(M = 1, T = -3)
    private val Compressibility = Dimension(L = 1, M = -1, T = 2)
    private val MagneticReluctance = Dimension(L = -2, M = -1, T = 2, I = 2)
    private val MagneticVectorPotential = Dimension(L = 1, M = 1, T = -2, I = -1)
    private val MagneticMoment = Dimension(L = 3, M = 1, T = -2, I = -1)
    private val MagneticRigidity = Dimension(L = 1, M = 1, T = -2, I = -1)
    private val RadiantExposure = Dimension(M = 1, T = -2)
    private val CatalyticEfficiency = Dimension(L = 3, T = -1, N = -1)
    private val MomentOfInertia = Dimension(L = 2, M = 1)
    private val SpecificAngularMomentum = Dimension(L = 2, T = -1)
    private val FrequencyDrift = Dimension(T = -2)
    private val LuminousEfficacy = Dimension(L = -2, M = -1, T = 3, J = 1)
    private val MagnetomotiveForce = Dimension(A = 1, I = 1)
    private val MagneticSusceptibility = Dimension(L = -1, M = -1, T = 2, I = 2)
    private val RadiantIntensity = Dimension(A = -2, L = 2, M = 1, T = -3)
    private val SpectralIntensity = Dimension(A = -2, L = 1, M = 1, T = -3)
    private val Radiance = Dimension(A = -2, M = 1, T = -3)
    private val SpectralRadiance = Dimension(A = -2, L = -1, M = 1, T = -3)
    private val SpectralPower = Dimension(L = 1, M = 1, T = -3)
    */

    val quantities = mutableSetOf<Quantity>()

    val units = mutableSetOf<UnitOfMeasure>()
}

val unitType = mutableMapOf(
    "Metre" to "Length",
    "Kilogram" to "Mass",
    "Second" to "Time",
    "Ampere" to "ElectricCurrent",
    "Kelvin" to "Temperature",
    "Mole" to "AmountOfSubstance",
    "Candela" to "LuminousIntensity",
    "Hertz" to "Frequency",
    "Radian" to "Angle",
    "Steradian" to "SolidAngle",
    "Newton" to "Force",
    "Pascal" to "Pressure",
    "Pascal" to "Stress",
    "Joule" to "Energy",
    "Joule" to "Work",
    "Joule" to "Heat",
    "Watt" to "Power",
    "Watt" to "RadiantFlux",
    "Coulomb" to "ElectricCharge",
    "Volt" to "ElectricalPotential",
    "Farad" to "ElectricalCapacitance",
    "Ohm" to "ElectricalResistance",
    "Siemens" to "ElectricalConductance",
    "Weber" to "MagneticFlux",
    "Tesla" to "MagneticFieldStrength",
    "Tesla" to "MagneticFluxDensity",
    "Henry" to "ElectricalInductance",
    "Lumen" to "LuminousFlux",
    "Lux" to "Illuminance",
    "Becquerel" to "Radioactivity",
    "Gray" to "AbsorbedDose",
    "Sievert" to "EquivalentDose",
    "Katal" to "CatalyticActivity"
).also { it.forEach { (k, v) -> it[v] = k } }.toMap()
val mathString = """
fun sin(x: `∠`) = Dimensionless(sin(x.siValue))
fun cos(x: `∠`) = Dimensionless(cos(x.siValue))
fun tan(x: `∠`) = Dimensionless(tan(x.siValue))
fun asin(x: Dimensionless) = `∠`(asin(x.siValue))
fun acos(x: Dimensionless) = `∠`(acos(x.siValue))
fun atan(x: Dimensionless) = `∠`(atan(x.siValue))
fun atan2(y: `L`, x: `L`) = `∠`(atan2(y.siValue, x.siValue))
fun hypot(x: `L`, y: `L`) = `L`(hypot(x.siValue, y.siValue))
fun sqrt(x: Dimensionless) = Dimensionless(sqrt(x.siValue))
fun exp(x: Dimensionless) = Dimensionless(exp(x.siValue))
fun expm1(x: Dimensionless, q: UomConverter<Dimensionless>) = q(expm1(q(x)))
fun log(x: Dimensionless, base: Dimensionless) = Dimensionless(log(x.siValue, base.siValue))
fun ln(x: Dimensionless, q: UomConverter<Dimensionless>) = q(ln(q(x)))
fun log10(x: Dimensionless, q: UomConverter<Dimensionless>) = q(log10(q(x)))
fun log2(x: Dimensionless, q: UomConverter<Dimensionless>) = q(log2(q(x)))
fun ln1p(x: Dimensionless, q: UomConverter<Dimensionless>) = q(ln1p(q(x)))
fun <Q : Quan<Q>> ceil(x: Q, q: UomConverter<Q>) = q(ceil(q(x)))
fun <Q : Quan<Q>> floor(x: Q, q: UomConverter<Q>) = q(floor(q(x)))
fun <Q : Quan<Q>> truncate(x: Q, q: UomConverter<Q>) = q(truncate(q(x)))
fun <Q : Quan<Q>> round(x: Q, q: UomConverter<Q>) = q(round(q(x)))
fun <Q : Quan<Q>> abs(x: Q) = x.abs
fun <Q : Quan<Q>> sign(x: Q) = x.signum
fun <Q : Quan<Q>> min(a: Q, b: Q) = a min b
fun <Q : Quan<Q>> max(a: Q, b: Q) = a max b
fun Dimensionless.pow(x: Dimensionless) = Dimensionless(siValue.pow(x.siValue))
fun <Q : Quan<Q>> Q.withSign(sign: Quan<*>) = new(siValue.withSign(sign.siValue))
val <Q : Quan<Q>> Q.ulp get() = new(siValue.ulp)
fun <Q : Quan<Q>> Q.nextUp() = new(siValue.nextUp())
fun <Q : Quan<Q>> Q.nextDown() = new(siValue.nextDown())
fun <Q : Quan<Q>> Q.nextTowards(to: Q) = new(siValue.nextTowards(to.siValue))
fun <Q : Quan<Q>> Q.roundToInt(q: UomConverter<Q>) = q(this).roundToInt()
fun <Q : Quan<Q>> Q.roundToLong(q: UomConverter<Q>) = q(this).roundToLong()
"""

infix fun <Q : Quan<Q>> Q.plusOrMinus(radius: Q) = (this - radius)..(this + radius)

operator fun <Q : Quan<Q>> Q.rangeTo(that: Q) = object : ClosedRange<Q> {
    override val start = min(that)
    override val endInclusive = max(that)
}

fun <Q : Quan<Q>> avg(a: Q, b: Q) = (a + b) / 2
fun <Q : Quan<Q>> avg(a: Q, b: Q, c: Q) = (a + b + c) / 3
fun <Q : Quan<Q>> avg(first: Q, vararg x: Q) = first.new(
    (first.siValue + x.sumByDouble(Quan<Q>::siValue)) /
            (1 + x.size)
)

operator fun <Q : Quan<Q>> Number.times(that: Quan<Q>): Q = that * this

fun <Q : Quan<Q>> Number.exa(f: UomConverter<Q>) = f(toDouble() * 1E18)
fun <Q : Quan<Q>> Q.exa(f: UomConverter<Q>) = f(this) * 1E-18

fun <Q : Quan<Q>> Number.peta(f: UomConverter<Q>) = f(toDouble() * 1E15)
fun <Q : Quan<Q>> Q.peta(f: UomConverter<Q>) = f(this) * 1E-15

fun <Q : Quan<Q>> Number.tera(f: UomConverter<Q>) = f(toDouble() * 1E12)
fun <Q : Quan<Q>> Q.tera(f: UomConverter<Q>) = f(this) * 1E-12

fun <Q : Quan<Q>> Number.giga(f: UomConverter<Q>) = f(toDouble() * 1E9)
fun <Q : Quan<Q>> Q.giga(f: UomConverter<Q>) = f(this) * 1E-9

fun <Q : Quan<Q>> Number.mega(f: UomConverter<Q>) = f(toDouble() * 1E6)
fun <Q : Quan<Q>> Q.mega(f: UomConverter<Q>) = f(this) * 1E-6

fun <Q : Quan<Q>> Number.kilo(f: UomConverter<Q>) = f(toDouble() * 1E3)
fun <Q : Quan<Q>> Q.kilo(f: UomConverter<Q>) = f(this) * 1E-3

fun <Q : Quan<Q>> Number.hecto(f: UomConverter<Q>) = f(toDouble() * 1E2)
fun <Q : Quan<Q>> Q.hecto(f: UomConverter<Q>) = f(this) * 1E-2

fun <Q : Quan<Q>> Number.deca(f: UomConverter<Q>) = f(toDouble() * 1E1)
fun <Q : Quan<Q>> Q.deca(f: UomConverter<Q>) = f(this) * 1E-1

fun <Q : Quan<Q>> Number.deci(f: UomConverter<Q>) = f(toDouble() * 1E-1)
fun <Q : Quan<Q>> Q.deci(f: UomConverter<Q>) = f(this) * 1E1

fun <Q : Quan<Q>> Number.centi(f: UomConverter<Q>) = f(toDouble() * 1E-2)
fun <Q : Quan<Q>> Q.centi(f: UomConverter<Q>) = f(this) * 1E2

fun <Q : Quan<Q>> Number.milli(f: UomConverter<Q>) = f(toDouble() * 1E-3)
fun <Q : Quan<Q>> Q.milli(f: UomConverter<Q>) = f(this) * 1E3

fun <Q : Quan<Q>> Number.micro(f: UomConverter<Q>) = f(toDouble() * 1E-6)
fun <Q : Quan<Q>> Q.micro(f: UomConverter<Q>) = f(this) * 1E6

fun <Q : Quan<Q>> Number.nano(f: UomConverter<Q>) = f(toDouble() * 1E-9)
fun <Q : Quan<Q>> Q.nano(f: UomConverter<Q>) = f(this) * 1E9

fun <Q : Quan<Q>> Number.pico(f: UomConverter<Q>) = f(toDouble() * 1E-12)
fun <Q : Quan<Q>> Q.pico(f: UomConverter<Q>) = f(this) * 1E12

fun <Q : Quan<Q>> Number.femto(f: UomConverter<Q>) = f(toDouble() * 1E-15)
fun <Q : Quan<Q>> Q.femto(f: UomConverter<Q>) = f(this) * 1E15

fun <Q : Quan<Q>> Number.atto(f: UomConverter<Q>) = f(toDouble() * 1E-18)
fun <Q : Quan<Q>> Q.atto(f: UomConverter<Q>) = f(this) * 1E18
package frc.kyberlib.math.threeD

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.numbers.*
import frc.kyberlib.math.units.extensions.Length
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.math.units.string

typealias TranslationVector = Matrix<N1, N3>

data class Translation3d(val x: Length, val y: Length, val z: Length) {
    val tVector: TranslationVector
        get() = MatBuilder(N1.instance, N3.instance).fill(x.meters, y.meters, z.meters)
    val poseTVector: Matrix<N1, N4>
        get() = MatBuilder(N1.instance, N4.instance).fill(x.meters, y.meters, z.meters, 1.0)

    constructor(translationVector: TranslationVector) : this(
        translationVector.get(0, 0).meters,
        translationVector.get(0, 1).meters,
        translationVector.get(0, 2).meters
    )

    fun transform(pose3d: Pose3d): Translation3d {
        return Translation3d(poseTVector.times(pose3d.matrix).block(1, 3, 0, 0))
    }

    fun rotate(rotation3d: Rotation3d): Translation3d {
        return Translation3d(tVector.times(rotation3d.matrix))
    }

    operator fun plus(other: Translation3d): Translation3d = Translation3d(x+other.x, y+other.y, z+other.z)
    operator fun minus(other: Translation3d): Translation3d = Translation3d(x-other.x, y-other.y, z-other.z)
    operator fun unaryMinus(): Translation3d = Translation3d(-x, -y, -z)

    override fun toString(): String = "(${x.string()}, ${y.string()}, ${z.string()})"
}
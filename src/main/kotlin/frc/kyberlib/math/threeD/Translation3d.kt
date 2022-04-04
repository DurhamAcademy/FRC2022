package frc.kyberlib.math.threeD

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.numbers.*
import frc.kyberlib.math.units.extensions.Length
import frc.kyberlib.math.units.extensions.meters

typealias TranslationVector = Matrix<N1, N4>

data class Translation3d(val x: Length, val y: Length, val z: Length) {
    val tVector: TranslationVector
        get() = MatBuilder(N1.instance, N4.instance).fill(x.meters, y.meters, z.meters, 1.0)

    constructor(translationVector: TranslationVector) : this(
        translationVector.get(0, 0).meters,
        translationVector.get(0, 1).meters,
        translationVector.get(0, 2).meters
    )

    fun transform(pose3d: Pose3d): Translation3d {
        return Translation3d(tVector.times(pose3d.matrix))
    }
}
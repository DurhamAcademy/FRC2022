package frc.kyberlib.math.threeD

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.numbers.N4

typealias PoseMatrix = Matrix<N4, N4>
data class Pose3d(val translation3d: Translation3d, val orientation: Rotation3d) {
    val matrix: PoseMatrix
        get() {
            val m = PoseMatrix(N4.instance, N4.instance)
            m.fill(0.0)
            m.assignBlock(0, 0, orientation.matrix)
            m.assignBlock(3, 0, translation3d.tVector)
            m.set(3, 3, 1.0)
            return m
        }

    constructor(poseMatrix: PoseMatrix) : this(
        Translation3d(poseMatrix.block(3, 1, 3, 0)),
        Rotation3d(poseMatrix.block(3, 3,0, 0))
        )

    fun transform(other: Pose3d): Pose3d {
        return Pose3d(matrix.times(other.matrix))
    }

}
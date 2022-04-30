package frc.kyberlib.math.threeD

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.numbers.N4
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.math.units.extensions.w

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

    operator fun plus(other: Pose3d) = Pose3d(matrix.times(other.matrix))
    operator fun minus(other: Pose3d) = Pose3d(matrix.times(other.matrix.inv()))
    operator fun unaryMinus() = Pose3d(matrix.inv())

    fun transform(other: Pose3d): Pose3d = this + other
    fun relativeTo(other: Pose3d) = this - other
}


fun main() {
    val spot = Translation3d(3.meters, 3.meters, 3.meters)
    val pose = Pose3d(spot, Rotation3d(90.degrees, 0.degrees, 0.degrees))
    val unitVector = Translation3d(1.meters, 0.meters, 0.meters)
    val rot2 = Rotation3d(45.degrees.w)
    println(unitVector.transform(pose))
    println(unitVector.rotate(rot2))
    println(spot + spot)
    println(spot.rotate(rot2))
}
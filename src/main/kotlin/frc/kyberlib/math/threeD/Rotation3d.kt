package frc.kyberlib.math.threeD

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N3
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.string
import kotlin.math.atan
import kotlin.math.pow
import kotlin.math.sqrt

typealias RotationMatrix = Matrix<N3, N3>

// http://planning.cs.uiuc.edu/node103.html
data class Rotation3d(val pitch: Angle, val yaw: Angle, val roll: Angle) {
    constructor(rotationMatrix: RotationMatrix) : this(
        atan(-rotationMatrix.get(3, 1)/ sqrt(rotationMatrix.get(3, 2).pow(2) + rotationMatrix.get(3, 3).pow(2))).radians,
        atan(rotationMatrix.get(2, 1)/rotationMatrix.get(1, 1)).radians,
        atan(rotationMatrix.get(3, 2)/rotationMatrix.get(3, 3)).radians
    )
    constructor(rotation2d: Rotation2d) : this(0.degrees, rotation2d.k, 0.degrees)
    val matrix: RotationMatrix
        get() = MatBuilder(N3.instance, N3.instance).fill(
                yaw.cos*pitch.cos, yaw.cos*pitch.sin*roll.sin-yaw.sin*roll.cos, yaw.cos*pitch.sin*roll.cos+yaw.sin*roll.sin,
                yaw.sin*pitch.cos, yaw.sin*pitch.sin*roll.sin+yaw.cos*roll.cos, yaw.sin*pitch.sin*roll.cos-yaw.cos*roll.sin,
                -pitch.sin, pitch.cos*roll.sin, pitch.cos*roll.cos
            )

    operator fun unaryMinus(): Rotation3d = Rotation3d(matrix.inv())
    operator fun plus(other: Rotation3d): Rotation3d {
        return Rotation3d(matrix.times(other.matrix))
    }
    operator fun minus(other: Rotation3d): Rotation3d {
        return Rotation3d(matrix.times(other.matrix.inv()))
    }

    override fun toString(): String = "(${pitch.string()}, ${yaw.string()}, ${roll.string()},)"
}
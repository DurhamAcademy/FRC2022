package frc.kyberlib.math

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.towards

data class PolarPosition(val r: Length, val theta: Angle) : Debug {
    fun cartesian(origin: Translation2d): Translation2d = Translation2d(theta.cos * r.meters, theta.sin * r.meters)
    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "r" to r, "theta" to theta
        )
    }
}

data class PolarPose(val r: Length, val theta: Angle, val orientation: Angle) : Debug {
    constructor(position: PolarPosition, orientation: Angle) : this(position.r, position.theta, orientation)
    fun cartesian(origin: Translation2d): Pose2d {
        val translation2d = Translation2d(origin.x + theta.cos * r.meters, origin.y + theta.sin * r.meters)
        return Pose2d(translation2d, origin.towards(translation2d).minus(orientation.w))
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "r" to r, "theta" to theta, "orientation" to orientation
        )
    }
}

data class PolarVelocity(val dr: LinearVelocity, val dTheta: AngularVelocity, val dOrientation: AngularVelocity) : Debug {
    fun cartesian(pose: PolarPose): ChassisSpeeds {
        val tangential = dTheta.toTangentialVelocity(pose.r)
        val heading = pose.orientation
        return ChassisSpeeds(
            dr.metersPerSecond * heading.cos + tangential.metersPerSecond * heading.sin,
            dr.metersPerSecond * heading.sin + tangential.metersPerSecond * heading.cos,
            dOrientation.radiansPerSecond
        )
    }

    fun cartesian(pose: Pose2d, origin: Translation2d): ChassisSpeeds {
        val tangential = dTheta.toTangentialVelocity(pose.translation.getDistance(origin).meters)
        val heading = pose.translation.towards(origin) - pose.rotation
        return ChassisSpeeds(
            dr.metersPerSecond * heading.cos + tangential.metersPerSecond * heading.sin,
            dr.metersPerSecond * heading.sin + tangential.metersPerSecond * heading.cos,
            dOrientation.radiansPerSecond
        )
    }
    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "dR" to dr, "dTheta" to dTheta, "dOrientation" to dOrientation
        )
    }

}

fun Translation2d.polar(origin: Translation2d): PolarPosition = PolarPosition(
    getDistance(origin).meters,
    origin.towards(this).k
)
fun Pose2d.polar(origin: Translation2d): PolarPose = PolarPose(
    translation.polar(origin),
    rotation.minus(translation.towards(origin)).k
)
fun ChassisSpeeds.polar(position: PolarPose): PolarVelocity {
    val forward = vxMetersPerSecond.metersPerSecond
    val strafe = vyMetersPerSecond.metersPerSecond
    val cos = position.orientation.cos
    val sin = position.orientation.sin
    return PolarVelocity(
        forward * cos + strafe * sin,
        -(forward * sin + strafe * cos) / position.r,
        omegaRadiansPerSecond.radiansPerSecond
    )
}

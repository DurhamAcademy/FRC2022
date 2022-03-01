package frc.kyberlib.math

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.towards

data class PolarPosition(val r: Length, val theta: Angle) {
    fun cartesian(origin: Translation2d): Translation2d = Translation2d(theta.cos * r.meters, theta.sin * r.meters)
}

data class PolarPose(val r: Length, val theta: Angle, val orientation: Angle) {
    constructor(position: PolarPosition, orientation: Angle) : this(position.r, position.theta, orientation)
    fun cartesian(origin: Translation2d): Pose2d {
        val translation2d = Translation2d(origin.x + theta.cos * r.meters, origin.y + theta.sin * r.meters)
        return Pose2d(translation2d, orientation.plus(origin.towards(translation2d)))
    }
}

data class PolarVelocity(val dr: LinearVelocity, val dTheta: AngularVelocity, val dOrientation: AngularVelocity)

fun Translation2d.polar(origin: Translation2d): PolarPosition = PolarPosition(
    getDistance(origin).meters,
    origin.towards(this).k
)
fun Pose2d.polar(origin: Translation2d): PolarPose = PolarPose(
    this.translation.polar(origin),
    origin.towards(this.translation).minus(this.rotation).k
)
fun ChassisSpeeds.polar(position: PolarPose): PolarVelocity {
    val forward = vxMetersPerSecond.metersPerSecond
    val strafe = vyMetersPerSecond.metersPerSecond
    val cos = position.orientation.cos
    val sin = position.orientation.sin
    return PolarVelocity(
        forward * cos + strafe * sin,
        (forward * sin + strafe * cos) / position.r,
        omegaRadiansPerSecond.radiansPerSecond
    )
}

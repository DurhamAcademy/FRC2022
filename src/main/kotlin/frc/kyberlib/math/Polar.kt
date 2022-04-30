package frc.kyberlib.math

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.string
import frc.kyberlib.math.units.towards
import frc.kyberlib.math.units.zeroTranslation
import kotlin.math.atan

data class PolarPosition(val r: Length, val theta: Angle) {
    fun cartesian(origin: Translation2d= zeroTranslation): Translation2d = Translation2d(theta.cos * r.meters, theta.sin * r.meters) + origin
}

data class PolarPose(val r: Length, val theta: Angle, val orientation: Angle) {
    constructor(position: PolarPosition, orientation: Angle) : this(position.r, position.theta, orientation)
    fun cartesian(origin: Translation2d = zeroTranslation): Pose2d {
        val translation2d = Translation2d(origin.x + theta.cos * r.meters, origin.y + theta.sin * r.meters)
        return Pose2d(translation2d, origin.towards(translation2d).minus(orientation.w))
    }

    override fun toString(): String = "PolarPose(${r.string()}, ${theta.string()}, ${orientation.string()})"
}

data class PolarVelocity(val dr: LinearVelocity, val dTheta: AngularVelocity, val dOrientation: AngularVelocity) {
    fun cartesian(pose: PolarPose): ChassisSpeeds {
        val tangential = dTheta.toTangentialVelocity(pose.r)
        val heading = pose.orientation
        return ChassisSpeeds(
            dr.metersPerSecond * heading.cos + tangential.metersPerSecond * heading.sin,
            dr.metersPerSecond * heading.sin + tangential.metersPerSecond * heading.cos,
            dOrientation.radiansPerSecond
        )
    }

    fun cartesian(pose: Pose2d, origin: Translation2d = zeroTranslation): ChassisSpeeds {
        val tangential = dTheta.toTangentialVelocity(pose.translation.getDistance(origin).meters)
        val heading = pose.translation.towards(origin) - pose.rotation
        return ChassisSpeeds(
            dr.metersPerSecond * heading.cos + tangential.metersPerSecond * heading.sin,
            dr.metersPerSecond * heading.sin + tangential.metersPerSecond * heading.cos,
            dOrientation.radiansPerSecond
        )
    }
    override fun toString(): String = "PolarVelocity(${dr.string()}, ${dTheta.string()}, ${dOrientation.string()})"
}

fun Translation2d.polar(origin: Translation2d = zeroTranslation): PolarPosition = PolarPosition(//1.meters, 0.degrees
    getDistance(origin).meters,
    origin.towards(this).k
)
fun Pose2d.polar(origin: Translation2d = zeroTranslation): PolarPose = PolarPose(
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
        (forward * sin + strafe * cos) / position.r,
        omegaRadiansPerSecond.radiansPerSecond
    )
}

@SuppressWarnings
fun main() {
    // unit tests //
    // pose
    println("pose test")
    val origin = Translation2d()
    val examplePose = PolarPose(5.meters, atan(3.0/4.0).radians, 45.degrees)
    println(examplePose.cartesian(origin))  // should be 4, 3, something - correct
    val otherTest = Pose2d(1.0, 1.0, 45.degrees.w)
    println(otherTest.polar(origin))  // should be √2/2 radians, √2 meters, 180º - correct

    // angularVelocity
    println("\nvel test")
    val exampleMoveLoc = PolarPose(1.meters, 0.degrees, 0.degrees)
    val polarVel = PolarVelocity(-1.metersPerSecond, 2.radiansPerSecond, 10.radiansPerSecond)
    println(polarVel.cartesian(exampleMoveLoc))
    val cartVel = ChassisSpeeds(-1.0, 2.0, 10.0)
    println(cartVel.polar(exampleMoveLoc))

    // speed tests
    println("\noptimization test")
    val s = Stopwatch()
    val testAmount = 10000000
    s.loop()
    println("pos")
    for(i in 0 until testAmount) {
        val blanks = Translation2d(1.0, 2.0)
    }
    println(s.loop())
    for(i in 0 until testAmount) {
        val blanks = Translation2d(1.0, 2.0).polar(origin)  // x7 time
    }
    println(s.loop())
    println("pose")
    for(i in 0 until testAmount) {
        val blanks = Pose2d(1.0, 2.0, Rotation2d(2.0))
    }
    println(s.loop())
    for(i in 0 until testAmount) {
        val blanks = Pose2d(1.0, 2.0, Rotation2d(2.0)).polar(origin)  // x7 time
    }
    println(s.loop())
    println("vel")
    for(i in 0 until testAmount) {
        val blanks = ChassisSpeeds(1.0, 1.0, 1.0)
    }
    println(s.loop())
    for(i in 0 until testAmount) {
        val blanks = ChassisSpeeds(1.0, 1.0, 1.0).polar(PolarPose(1.meters, 2.radians, 2.radians))  // x7 time
    }
    println(s.loop())
}

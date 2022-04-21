package frc.robot.subsystems

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Game
import frc.kyberlib.math.randomizer
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.towards
import frc.kyberlib.math.units.transform
import frc.kyberlib.math.units.zeroPose
import frc.robot.Constants
import frc.robot.RobotContainer
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import org.photonvision.targeting.TargetCorner
import kotlin.math.atan
import kotlin.math.pow
import kotlin.math.sqrt

object Limelight : SubsystemBase() {
    // important public values
    val visionDistance: Length?  // how far based on vision (null if not found)
        get() = visionPitch?.let { pitch -> 2.feet + distanceFilter.calculate((Constants.UPPER_HUB_HEIGHT.inches - 1 - Constants.LIMELIGHT_HEIGHT.inches) / (Constants.LIMELIGHT_ANGLE + pitch).tan).inches }  // this could be wrong
    inline val distance  // distance either by vision or nav (never null)
        get() = visionDistance ?: RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION).meters
    val visionOffset: Angle?  // how far to side turret is from target
        get() = target?.yaw?.let { visionFilter.calculate(-it).degrees }
    var targetVisible: Boolean = false  // currently has target
        private set
    var movementAngleOffset: Angle = 0.degrees  // how much robot movement should cause turret to turn to compensate
    var effectiveDistance: Length = 0.meters  // how far shoooter should behave to compensate for movement

    // util values for calculations
    private val visionFilter: LinearFilter = LinearFilter.singlePoleIIR(.07, .02)
    private var latestResult: PhotonPipelineResult? = null // latest data from limelight
    private var target: PhotonTrackedTarget? = null  // target result
    private val visionPitch: Angle?  // how far up the target is
        get() = target?.pitch?.degrees

    private val distanceFilter = LinearFilter.movingAverage(8) // how far from the center of the hub the robot is (based on limelight)

    private const val moveIterations = 1  // how many times to optimize the shoot while move target

    private fun timeOfFlight(dis: Length) = Constants.TIME_OF_FLIGHT_INTERPOLATOR.calculate(dis.meters)

    override fun periodic() {
        if (Game.sim) {  // simulate vision
            val boundRect = listOf(TargetCorner(100.0, 100.0), TargetCorner(200.0, 100.0), TargetCorner(200.0, 200.0), TargetCorner(100.0, 200.0))
            val off = (RobotContainer.navigation.position.towards(Constants.HUB_POSITION).k - Turret.fieldRelativeAngle + randomizer.nextGaussian().degrees * 0.0).normalized.let { if (it > 0.5.rotations) it - 1.rotations else it }
            val target = if (off.absoluteValue < 25.degrees) listOf(PhotonTrackedTarget(
                -off.degrees,
                atan((Constants.UPPER_HUB_HEIGHT - Constants.LIMELIGHT_HEIGHT - 1.inches) / (RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION).meters - 2.feet)).radians.degrees,
                1.0, 1.0, zeroPose.transform, boundRect
            )) else listOf()
            RobotContainer.limelight.submitProcessedFrame(20.0, target)
        }
        // update vars with vision data
        latestResult = RobotContainer.limelight.latestResult
        targetVisible = latestResult != null && latestResult!!.hasTargets()
        target = latestResult?.let { if (it.hasTargets()) it.bestTarget else null }

        if (RobotContainer.op.shootWhileMoving) {
            // update shooting while moving values
            var r = distance
            val hubSpeeds = Drivetrain.hubRelativeSpeeds
            val parallel = hubSpeeds.vxMetersPerSecond
            val perp = hubSpeeds.vyMetersPerSecond
            for (i in 0 until moveIterations) {
                val time = timeOfFlight(r) * SmartDashboard.getNumber("time mult", 1.0)
                val a = r.meters - parallel * time
                val b = perp * time
                r = sqrt(a.pow(2) + b.pow(2)).meters.absoluteValue
                movementAngleOffset = atan(b / a).radians
            }
            effectiveDistance = r
        }
        else {
            effectiveDistance = distance
            movementAngleOffset = 0.degrees
        }
    }
}
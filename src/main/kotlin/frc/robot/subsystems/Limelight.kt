package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Game
import frc.kyberlib.math.randomizer
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.towards
import frc.kyberlib.math.units.transform
import frc.kyberlib.math.units.zeroPose
import frc.robot.Constants
import frc.robot.RobotContainer
import org.photonvision.targeting.PhotonTrackedTarget
import org.photonvision.targeting.TargetCorner
import kotlin.math.atan

object Limelight : SubsystemBase() {
    // latest data from limelight
    var targetVisible: Boolean = false  // currently has target
        private set
    private var target: PhotonTrackedTarget? = null  // target result
    val visionOffset: Angle?  // how far to side turret is from target
        get() = target?.yaw?.degrees
    private val visionPitch: Angle?  // how far up the target is
        get() = target?.pitch?.degrees

    // how far from the center of the hub the robot is (based on limelight)
    val visionDistance: Length?
        get() = visionPitch?.let { pitch -> 2.feet + ((Constants.UPPER_HUB_HEIGHT.inches - 1 - Constants.LIMELIGHT_HEIGHT.inches) / (Constants.LIMELIGHT_ANGLE + pitch).tan).inches } // this could be wrong
    val distance
        inline get() = visionDistance ?: RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION).meters
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
        val latestResult = RobotContainer.limelight.latestResult
        targetVisible = latestResult != null && latestResult.hasTargets()
        target = latestResult?.let { if (it.hasTargets()) it.bestTarget else null }
    }
}
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
    private val visionFilter: LinearFilter = LinearFilter.singlePoleIIR(.07, .02)

    // latest data from limelight
    var targetVisible: Boolean = false  // currently has target
        private set
    val visionOffset: Angle  // how far to side turret is from target
        get() = RobotContainer.limelight.yaw.degrees.let { visionFilter.calculate(-it).degrees }
    private val visionPitch: Angle  // how far up the target is
        get() = RobotContainer.limelight.pitch

    // how far from the center of the hub the robot is (based on limelight)
    private val distanceFilter = LinearFilter.movingAverage(8)
    val visionDistance: Length
        get() = 2.feet + distanceFilter.calculate((Constants.UPPER_HUB_HEIGHT.inches - 1 - Constants.LIMELIGHT_HEIGHT.inches) / (Constants.LIMELIGHT_ANGLE + visionPitch).tan).inches  // this could be wrong
    val distance
        inline get() = visionDistance ?: RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION).meters

    private const val moveIterations = 1  // how many times to optimize the shoot while move target
    var movementAngleOffset: Angle = 0.degrees  // how much robot movement should cause turret to turn to compensate
    var effectiveDistance: Length = 0.meters  // how far shoooter should behave to compensate for movement

    private fun timeOfFlight(dis: Length) = Constants.TIME_OF_FLIGHT_INTERPOLATOR.calculate(dis.meters)

    val estimatedOffset
        inline get() = (RobotContainer.navigation.position.towards(Constants.HUB_POSITION).k - RobotContainer.navigation.heading).normalized.let { if (it > 0.5.rotations) it - 1.rotations else it }


    override fun periodic() {
        effectiveDistance = distance
        movementAngleOffset = 0.degrees

        if (Game.sim) {  // simulate vision
            val boundRect = listOf(TargetCorner(100.0, 100.0), TargetCorner(200.0, 100.0), TargetCorner(200.0, 200.0), TargetCorner(100.0, 200.0))
            val off = estimatedOffset
            val target = if (off.absoluteValue < 25.degrees) listOf(PhotonTrackedTarget(
                -off.degrees,
                atan((Constants.UPPER_HUB_HEIGHT - Constants.LIMELIGHT_HEIGHT - 1.inches) / (RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION).meters - 2.feet)).radians.degrees,
                1.0, 1.0, zeroPose.transform, boundRect
            )) else listOf()
            RobotContainer.limelight.submitProcessedFrame(20.0, target)
        }
        // update vars with vision data
        targetVisible = RobotContainer.limelight.targetFound
    }
}
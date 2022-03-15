package frc.robot.subsystems

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.command.Game
import frc.kyberlib.math.filters.Differentiator
import frc.kyberlib.math.randomizer
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.towards
import frc.kyberlib.math.zeroIf
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.motorcontrol.KMotorController.StateSpace.systemLoop
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.simulation.field.KField2d
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.turret.SeekTurret
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget


/**
 * Status of what the turret is doing
 */
enum class TurretStatus {
    LOCKED, ADJUSTING, FROZEN, NOT_FOUND, LOST
}
/**
 * Controls the turret
 */
object Turret : SubsystemBase(), Debug {
    var status = TurretStatus.LOST
    override val priority: DebugFilter = DebugFilter.Max

    // characterization of the turret
    private val feedforward = SimpleMotorFeedforward(0.22832, 1.1432, 0.045857)
    // actual turret motors
    val turret = KSparkMax(11).apply {
        identifier = "turret"
        gearRatio = Constants.TURRET_GEAR_RATIO
        motorType = DCMotor.getNeo550(1)
        brakeMode = true

        val headingDiff = Differentiator()

        PID = ProfiledPIDController(70.0, 2.0, 0.0, TrapezoidProfile.Constraints(3.0, 2.0)).apply {
            setIntegratorRange(-2.0, 2.0)
        }  // these constraints are not tested on real

        val new = { it: KMotorController ->
            val rot = -headingDiff.calculate(RobotContainer.gyro.heading.value).radiansPerSecond * 0.1.seconds
            position = clampSafePosition(it.positionSetpoint + rot)

            val offsetCorrection = PID.calculate(position.rotations, positionSetpoint.rotations)
            val ff = feedforward.calculate(PID.setpoint.velocity.rotationsPerSecond.radiansPerSecond)
            (ff + offsetCorrection)//.coerceIn(-4.0, 4.0)
        }
        customControl = new

        if(Game.sim) setupSim(feedforward)
    }

    // angle of the turret from top view
    var fieldRelativeAngle: Angle
        get() = (turret.position + RobotContainer.navigation.heading).normalized
        set(value) {
            turret.position = value - RobotContainer.navigation.heading
        }

    init {
        defaultCommand = SeekTurret
//        log("SeekTurret as default is off until built", logMode = LogMode.WARN)
    }

    /**
     * Makes an angle safe for the electronics to not get tangled
     */
    private fun clampSafePosition(angle: Angle): Angle {
        return (angle + 25.degrees).normalized - 25.degrees
    }

    /**
     * Allows the turret to know its position relative to everything else
     */
    fun zeroTurret() {  // zeros the robot position to looking straight aheead
        turret.resetPosition()
    }

    private var latestResult: PhotonPipelineResult? = null
    var targetVisible: Boolean = false
        private set
    var lost = false
    private var target: PhotonTrackedTarget? = null
    val visionOffset: Angle?
        get() = if (Game.real) target?.yaw?.let { -it.degrees }
                else {
                    var off = (RobotContainer.navigation.position.towards(Constants.HUB_POSITION).k - fieldRelativeAngle + randomizer.nextGaussian().degrees * 0.0)
                    off = off.normalized
                    if (off > 0.5.rotations) off -= 1.rotations
                    if(off.absoluteValue < 20.degrees) off.degrees.degrees else null
//                    off
                }
    private val visionPitch: Angle?
        get() = target?.pitch?.degrees

    // how far from the center of the hub the robot is (based on limelight)
    private val distanceFilter = LinearFilter.movingAverage(8)
    val targetDistance: Length?
        get() = if (Game.sim) RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION).meters + randomizer.nextGaussian().inches
                else visionPitch?.let { pitch -> 2.feet + distanceFilter.calculate((Constants.UPPER_HUB_HEIGHT.inches - 1 - Constants.LIMELIGHT_HEIGHT.inches) / (Constants.LIMELIGHT_ANGLE + pitch).tan).inches}  // this could be wrong

    val ready: Boolean
        get() = (targetVisible && visionOffset!!.absoluteValue < Constants.TURRET_TOLERANCE) || Game.sim || status == TurretStatus.FROZEN

    fun reset() {
        distanceFilter.reset()
        turret.PID.reset(turret.position.rotations, turret.velocity.rotationsPerSecond)
    }

    private val lostDebouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)
    override fun periodic() {
        debugDashboard()
        if(Game.real) {
            latestResult = RobotContainer.limelight.latestResult
            targetVisible = latestResult != null && latestResult!!.hasTargets()
            lost = !lostDebouncer.calculate(targetVisible)
            target = latestResult?.let { if(it.hasTargets()) it.bestTarget else null  }
        }
        else {
            targetVisible = visionOffset != null
            lost = !lostDebouncer.calculate(targetVisible)
        }
        turret.updateVoltage()
    }

    override fun simulationPeriodic() {
        KField2d.getObject("turret").pose = Pose2d(RobotContainer.navigation.position, fieldRelativeAngle.w)
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "turret" to turret,
            "turret error" to visionOffset,
            "field Heading" to fieldRelativeAngle.radians,
            "target detected" to targetVisible
        )
    }
}
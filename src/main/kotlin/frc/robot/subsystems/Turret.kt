package frc.robot.subsystems

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.math.filters.Differentiator
import frc.kyberlib.math.randomizer
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.towards
import frc.kyberlib.math.zeroIf
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.simulation.field.KField2d
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.turret.SeekTurret
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.absoluteValue


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

    //    override val priority: DebugFilter = DebugFilter.Max
    private val visionFilter: LinearFilter = LinearFilter.singlePoleIIR(.07, .02)

    var isZeroed = false

    // characterization of the turret
    private val feedforward = SimpleMotorFeedforward(0.05, 1.1432, 0.045857) // 0.22832

    // actual turret motors
    val controller = ProfiledPIDController(40.0, 3.0, 0.0, TrapezoidProfile.Constraints(1.0, 1.0)).apply {
        setIntegratorRange(-2.0, 2.0)
    }  // these constraints are not tested on real
    val turret = KSparkMax(11).apply {
        identifier = "turret"
        gearRatio = Constants.TURRET_GEAR_RATIO
        motorType = DCMotor.getNeo550(1)
        brakeMode = true
        currentLimit = 15

        val headingDiff = Differentiator()
        val new = {
            val polarSpeeds = Drivetrain.polarSpeeds
            val rot =
                polarSpeeds.dTheta * 0.1.seconds//-headingDiff.calculate(RobotContainer.gyro.heading.value).radiansPerSecond * 0.1.seconds
            val mov = polarSpeeds.dOrientation * 0.0.seconds
            position = clampSafePosition(positionSetpoint + rot + mov)

            val offsetCorrection = controller.calculate(position.rotations, positionSetpoint.rotations)
            val ff = feedforward.calculate(controller.setpoint.velocity.rotationsPerSecond.radiansPerSecond)
            if (isZeroed) (ff + offsetCorrection) else 0.0//.coerceIn(-4.0, 4.0)
        }

        val good = {
            position = clampSafePosition(positionSetpoint)
            val polarSpeeds = Drivetrain.polarSpeeds
            val movementComp = -polarSpeeds.dTheta
            val chassisComp = polarSpeeds.dOrientation
            val offsetCorrection = controller.calculate(positionError.rotations).rotationsPerSecond / feedforward.kv
            val targetVelocity = offsetCorrection - chassisComp - movementComp
            val v = feedforward.calculate(targetVelocity.radiansPerSecond)// + controller.calculate(velocityError.radiansPerSecond)
            v.zeroIf { v.absoluteValue < 1.0 }
        }

        val loop = KMotorController.StateSpace.systemLoop(
            positionSystem(feedforward),  // degrees units
            5.0,
            0.01,
            1.0,
            2.0,
            12.0
        )

        val state = { _: KMotorController ->
            position = clampSafePosition(positionSetpoint)
            SmartDashboard.putNumber("error", positionError.degrees)
            SmartDashboard.putNumber("set", positionSetpoint.degrees)
            val polarSpeeds = Drivetrain.polarSpeeds
            val compSpeed = (-polarSpeeds.dTheta * 0.0 - polarSpeeds.dOrientation).degreesPerSecond
            loop.nextR = VecBuilder.fill(
                positionSetpoint.degrees,
                compSpeed * 0.03
            )
            loop.correct(VecBuilder.fill(position.degrees))
            loop.predict(updateRate.seconds)  // math
            val nextVoltage = loop.getU(0)  // input
            if (isZeroed) nextVoltage else 0.0
        }

        customControl = state

        if (Game.sim) setupSim(feedforward)
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
        return (angle + 30.degrees).normalized - 30.degrees
    }

    /**
     * Allows the turret to know its position relative to everything else
     */
    fun zeroTurret() {  // zeros the robot position to looking straight aheead
        turret.resetPosition()
    }

    // latest data from limelight
    private var latestResult: PhotonPipelineResult? = null
    var targetVisible: Boolean = false  // currently has target
        private set
    var lost = false  // should the turret behave as lost
    private var target: PhotonTrackedTarget? = null  // target result
    val visionOffset: Angle?  // how far to side turret is from target
        get() = if (Game.real) target?.yaw?.let { visionFilter.calculate(-it).degrees }
        else {
            var off =
                (RobotContainer.navigation.position.towards(Constants.HUB_POSITION).k - fieldRelativeAngle + randomizer.nextGaussian().degrees * 0.0)
            off = off.normalized
            if (off > 0.5.rotations) off -= 1.rotations
            if (true || off.absoluteValue < 20.degrees) off.degrees.degrees else null
//                    off
        }
    private val visionPitch: Angle?  // how far up the target is
        get() = target?.pitch?.degrees

    // how far from the center of the hub the robot is (based on limelight)
    private val distanceFilter = LinearFilter.movingAverage(8)
    val targetDistance: Length?
        get() = if (Game.sim) RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION).meters + randomizer.nextGaussian().inches
        else visionPitch?.let { pitch -> 2.feet + distanceFilter.calculate((Constants.UPPER_HUB_HEIGHT.inches - 1 - Constants.LIMELIGHT_HEIGHT.inches) / (Constants.LIMELIGHT_ANGLE + pitch).tan).inches }  // this could be wrong

    val ready: Boolean  // is the Turret prepared to shoot
        get() = Game.sim || (targetVisible && turret.positionError.absoluteValue < Constants.TURRET_TOLERANCE) || status == TurretStatus.FROZEN

    fun reset() {  // clear stuff to remove weird things happening when switching between commands
        distanceFilter.reset()
        controller.reset(turret.position.rotations, turret.velocity.rotationsPerSecond)
    }

    // smooths out how fast we switch between lost and found
    private val lostDebouncer = Debouncer(0.2, Debouncer.DebounceType.kBoth)  // *** this was changed from .5
    override fun periodic() {
        SmartDashboard.putString("turret cmd", this.currentCommand?.name ?: "none")
        debugDashboard()
        // update vars with vision data
        if (Game.real) {
            latestResult = RobotContainer.limelight.latestResult
            targetVisible = latestResult != null && latestResult!!.hasTargets()
            lost = !lostDebouncer.calculate(targetVisible)
            target = latestResult?.let { if (it.hasTargets()) it.bestTarget else null }
        } else {
            targetVisible = visionOffset!!.absoluteValue > 20.degrees
            lost = !lostDebouncer.calculate(targetVisible)
        }
        if (status != TurretStatus.FROZEN) turret.updateVoltage()
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
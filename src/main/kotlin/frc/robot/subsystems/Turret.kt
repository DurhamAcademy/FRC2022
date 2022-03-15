package frc.robot.subsystems

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.filter.MedianFilter
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
import frc.kyberlib.motorcontrol.KSimulatedESC
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
    // some filters
//    private val visionFilter: LinearFilter = LinearFilter.movingAverage(1)
//    private val positionFilter = MedianFilter(5)
    // actual turret motors
    val turret = KSparkMax(11).apply {
        identifier = "turret"
        kP = .8
        kD = 0.01
//        maxAcceleration = 2.radiansPerSecond
//        maxVelocity = 3.radiansPerSecond
        gearRatio = Constants.TURRET_GEAR_RATIO
        motorType = DCMotor.getNeo550(1)

        val headingDiff = Differentiator()

        val pid2020 = ProfiledPIDController(20.0, 4.0, 0.0, TrapezoidProfile.Constraints(3.0, 1.0)).apply {
            setIntegratorRange(-3.0, 3.0)
        }
        val oldControls = { it: KMotorController ->
            position = clampSafePosition(it.positionSetpoint)
            pid2020.calculate(position.rotations, positionSetpoint.rotations)
        }

        val posLoop = systemLoop(positionSystem(feedforward), 3.0, 0.1, 2.degrees.value, 10.degreesPerSecond.value)
        val posSS = { it: KMotorController ->
            position = clampSafePosition(positionSetpoint)
            posLoop.nextR = VecBuilder.fill(positionSetpoint.radians, velocitySetpoint.radiansPerSecond)  // r = reference (setpoint)
            posLoop.correct(VecBuilder.fill(position.radians))  // update with empirical
            posLoop.predict(0.02)  // math
            val v = posLoop.getU(0)  // input
            v
        }

        val offsetCorrector = ProfiledPIDController(3.0, 0.0, 0.0002, TrapezoidProfile.Constraints(3.0, 1.0)).apply {
            setTolerance(Constants.TURRET_DEADBAND.radians)
            setIntegratorRange(-1.0, 1.0)
        }
        val velLoop = systemLoop(velocitySystem(feedforward), 3.0, 0.1, 10.degreesPerSecond.value, 10.0)
        val velSS = {it: KMotorController ->
            val polarSpeeds = Drivetrain.polarSpeeds
            val movementComp = -polarSpeeds.dTheta
//            val chassisComp = headingDiff.calculate(Navigator.instance!!.heading.radians).radiansPerSecond
            val chassisComp = polarSpeeds.dOrientation
            val setpoint = clampSafePosition(it.positionSetpoint)
            val offsetCorrection = offsetCorrector.calculate(it.position.value, setpoint.value).radiansPerSecond
            val targetVelocity = offsetCorrection - chassisComp - movementComp * 0.0
            velLoop.nextR = VecBuilder.fill(targetVelocity.radiansPerSecond)  // r = reference (setpoint)
            velLoop.correct(VecBuilder.fill(velocity.radiansPerSecond))  // update with empirical
            velLoop.predict(0.02)  // math
            val v = velLoop.getU(0)  // input
            v
        }

        val velocityControls = { it: KMotorController ->
            val polarSpeeds = Drivetrain.polarSpeeds
            val movementComp = -polarSpeeds.dTheta
            val chassisComp = polarSpeeds.dOrientation
            val setpoint = clampSafePosition(it.positionSetpoint)
            val offset = it.position - setpoint
            val offsetCorrection = offsetCorrector.calculate(position.value, setpoint.value).radiansPerSecond
            val targetVelocity = offsetCorrection - chassisComp - movementComp
            val v = feedforward.calculate(targetVelocity.radiansPerSecond)// + it.PID.calculate(velocityError.radiansPerSecond)
            if (offset < Constants.TURRET_TOLERANCE) status = TurretStatus.LOCKED
            v.zeroIf { it < 0.5 }
        }

        val garbage = ProfiledPIDController(70.0, 8.0, 0.0, TrapezoidProfile.Constraints(3.0, 1.0)).apply {
            setIntegratorRange(-6.0, 6.0)
        }  // these constraints are not tested on real
        val notGarbage = { it: KMotorController ->
            val polarSpeeds = Drivetrain.polarSpeeds
            val movementComp = -polarSpeeds.dTheta
            val chassisComp = polarSpeeds.dOrientation
            position = clampSafePosition(it.positionSetpoint)
            val offsetCorrection = garbage.calculate(position.rotations, positionSetpoint.rotations)
            val targetVelocity = (- chassisComp - movementComp).radiansPerSecond.zeroIf { it < 0.2 }
            val velComp = feedforward.calculate(targetVelocity) + it.PID.calculate(position.rotations, positionSetpoint.rotations)
            val v = velComp + offsetCorrection
            v
        }

        val new = { it: KMotorController ->
            val rot = -headingDiff.calculate(RobotContainer.gyro.heading.value).radiansPerSecond * 0.3.seconds
            position = clampSafePosition(it.positionSetpoint + rot)

            val offsetCorrection = garbage.calculate(position.rotations, positionSetpoint.rotations)
            val ff = feedforward.calculate(it.PID.setpoint.velocity.rotationsPerSecond.radiansPerSecond)
            ff + offsetCorrection
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
    fun clampSafePosition(angle: Angle): Angle {
//        return  if(angle.value.between(minAngle.value, maxAngle.value)) angle
//                else if (angle.normalized.value.between(minAngle.value, maxAngle.value)) angle.normalized
//                else angle.coerceIn(minAngle, maxAngle)
        return (angle + 15.degrees).normalized - 15.degrees//positionFilter.calculate(angle.normalized.value).radians
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
    private var target: PhotonTrackedTarget? = null
    val visionOffset: Angle?
        get() = if (Game.real) target?.yaw?.let { -it.degrees }
                else {
                    var off = (RobotContainer.navigation.position.towards(Constants.HUB_POSITION).k - fieldRelativeAngle + randomizer.nextGaussian().degrees * 0.0)
                    off = off.normalized
                    if (off > 0.5.rotations) off -= 1.rotations
//                    println("robot: ${fieldRelativeAngle.string()}, towards: ${RobotContainer.navigation.position.towards(Constants.HUB_POSITION).k.string()}")
//                    println(off.string())
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
        get() = turret.positionError.absoluteValue < Constants.TURRET_TOLERANCE || status == TurretStatus.FROZEN

    override fun periodic() {
        debugDashboard()
        if(Game.real) {
            latestResult = RobotContainer.limelight.latestResult
            targetVisible = latestResult != null && latestResult!!.hasTargets()
            target = latestResult?.let { if(it.hasTargets()) it.bestTarget else null  }
        }
        else {
            targetVisible = visionOffset != null
        }
//        turret.updateVoltage()
//        SmartDashboard.putNumber("tangle", turret.position.degrees)
//        SmartDashboard.putNumber("voff", visionOffset?.degrees ?: 0.0)
//        SmartDashboard.putBoolean("hall", RobotContainer.turretLimit.get())
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
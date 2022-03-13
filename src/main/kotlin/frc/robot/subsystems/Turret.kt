package frc.robot.subsystems

import edu.wpi.first.math.StateSpaceUtil
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.filter.MedianFilter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.math.between
import frc.kyberlib.math.filters.Differentiator
import frc.kyberlib.math.randomizer
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.towards
import frc.kyberlib.math.zeroIf
import frc.kyberlib.motorcontrol.KMotorController.StateSpace.systemLoop
import frc.kyberlib.motorcontrol.KSimulatedESC
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
enum class TURRET_STATUS {
    LOCKED, ADJUSTING, NOT_FOUND, LOST, FROZEN
}


/**
 * Controls the turret
 */
object Turret : SubsystemBase(), Debug {
    var status = TURRET_STATUS.LOST

    // characterization of the turret
    private val feedforward = SimpleMotorFeedforward(0.57083, 1.2168, 0.036495)
    private val visionFilter: LinearFilter = LinearFilter.movingAverage(10)
    private val positionFilter = MedianFilter(5)
    // actual turret motors
    val turret = KSimulatedESC(11).apply {
        identifier = "turret"
        kP = 1.2
        kD = 0.01
        maxAcceleration = 2.radiansPerSecond
        maxVelocity = 3.radiansPerSecond
        gearRatio = Constants.TURRET_GEAR_RATIO
        motorType = DCMotor.getNeo550(1)

        val loop = systemLoop(velocitySystem(feedforward), 3.0, 0.1, 10.degreesPerSecond.value, 10.0)

        val offsetCorrector = ProfiledPIDController(2.0, 0.0, 0.0002, TrapezoidProfile.Constraints(2.0, 1.0)).apply {
            setTolerance(Constants.TURRET_DEADBAND.radians)
            setIntegratorRange(-1.0, 1.0)
         }
        val headingDiff = Differentiator()
        customControl = {
            val polarSpeeds = Drivetrain.polarSpeeds
            val movementComp = -polarSpeeds.dTheta
//            val chassisComp = headingDiff.calculate(Navigator.instance!!.heading.radians).radiansPerSecond
            val chassisComp = polarSpeeds.dOrientation
            val setpoint = clampSafePosition(it.positionSetpoint)
            val offset = it.position - setpoint
            val offsetCorrection = offsetCorrector.calculate(offset.radians).radiansPerSecond
            val targetVelocity = offsetCorrection - chassisComp - movementComp * 0.0
//            loop.nextR = VecBuilder.fill(targetVelocity.radiansPerSecond)  // r = reference (setpoint)
//            loop.correct(VecBuilder.fill(velocity.radiansPerSecond))  // update with empirical
//            loop.predict(0.02)  // math
//            val v = loop.getU(0)  // input
            SmartDashboard.putNumber("off", offsetCorrection.degreesPerSecond)
            SmartDashboard.putNumber("chas", -chassisComp.degreesPerSecond)
            SmartDashboard.putNumber("mov", -movementComp.degreesPerSecond)
            SmartDashboard.putNumber("tar", targetVelocity.degreesPerSecond)
            val velocityError = it.velocity - targetVelocity
            val v = feedforward.calculate(targetVelocity.radiansPerSecond)// + it.PID.calculate(velocityError.radiansPerSecond)
            if (offset < Constants.TURRET_TOLERANCE) status = TURRET_STATUS.LOCKED
            v.zeroIf { v.absoluteValue < 1.0  }
        }

        if(Game.sim) setupSim(feedforward)
    }

    // angle of the turret from top view
    var fieldRelativeAngle: Angle
        get() = turret.position + RobotContainer.navigation.heading.normalized
        set(value) {
            turret.position = value - RobotContainer.navigation.heading
        }

    init {
        defaultCommand = SeekTurret
//        log("SeekTurret as default is off until built", logMode = LogMode.WARN)
    }

//    val minAngle = 0.degrees
//    val maxAngle = 270.degrees
    /**
     * Makes an angle safe for the electronics to not get tangled
     */
    private fun clampSafePosition(angle: Angle): Angle {
//        return  if(angle.value.between(minAngle.value, maxAngle.value)) angle
//                else if (angle.normalized.value.between(minAngle.value, maxAngle.value)) angle.normalized
//                else angle.coerceIn(minAngle, maxAngle)
        return positionFilter.calculate(angle.normalized.value).radians
    }

    /**
     * Allows the turret to know its position relative to everything else
     */
    fun zeroTurret() {  // zeros the robot position to looking straight aheead
        turret.resetPosition()
    }

    private val latestResult: PhotonPipelineResult?
        get() = RobotContainer.limelight.latestResult
    val targetVisible: Boolean 
        get() = (Game.sim && visionOffset != null) || (latestResult != null && latestResult!!.hasTargets() && visionOffset != null)
    private val target: PhotonTrackedTarget?
        get() = latestResult?.bestTarget

    val visionOffset: Angle?
        get() = if (Game.real) target?.yaw?.let { visionFilter.calculate(-it).degrees }
                else {
                    val off = (RobotContainer.navigation.position.towards(Constants.HUB_POSITION).k - fieldRelativeAngle
                            + randomizer.nextGaussian().degrees * 2.0)
                    if(off.degrees.absoluteValue < 20.0 || true) visionFilter.calculate(off.degrees).degrees else null
                }

    val visionPitch: Angle?
        get() = target?.pitch?.degrees

    val readyToShoot
        get() = turret.positionError < Constants.TURRET_TOLERANCE

    override fun periodic() {
        debugDashboard()
        SmartDashboard.putNumber("tangle", turret.position.degrees)
        SmartDashboard.putNumber("voff", visionOffset?.degrees ?: 0.0)
        SmartDashboard.putBoolean("hall", RobotContainer.turretLimit.get())
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
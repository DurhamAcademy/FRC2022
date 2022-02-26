package frc.robot.subsystems

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.MedianFilter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.kyberlib.command.Game
import frc.kyberlib.command.KSubsystem
import frc.kyberlib.math.units.extensions.Angle
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.k
import frc.kyberlib.math.units.extensions.radiansPerSecond
import frc.kyberlib.math.units.towards
import frc.kyberlib.motorcontrol.KMotorController.StateSpace.systemLoop
import frc.kyberlib.motorcontrol.KSimulatedESC
import frc.kyberlib.simulation.field.KField2d
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.turret.AimTurret
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget


/**
 * Status of what the turret is doing
 */
enum class TURRET_STATUS {
    LOCKED, ADJUSTING, NOT_FOUND, LOST
}


/**
 * Controls the turret
 */
object Turret : KSubsystem() {
    init {
        log("init")
    }
    var status = TURRET_STATUS.LOST

    // characterization of the turret
    private val feedforward = SimpleMotorFeedforward(0.94765, 1.2215, 0.05002)
    val visionFilter = MedianFilter(8)
    // actual turret motors
    val turret = KSimulatedESC(30).apply {
        kP = 0.0
        kD = 0.0
        gearRatio = Constants.TURRET_GEAR_RATIO
        motorType = DCMotor.getNeo550(1)
        val loop = systemLoop(positionSystem(feedforward), 3.0, 0.1, 3.degrees.radians, 10.0)

        // fancy control
        val offsetCorrector = ProfiledPIDController(0.3, 0.0, 0.0, TrapezoidProfile.Constraints(12.0, 4.0)).apply {
            setTolerance(Constants.TURRET_DEADBAND.radians)
            setIntegratorRange(-1.0, 1.0)
         }
        customControl = {
            val polarSpeeds = Drivetrain.polarSpeeds
            val movementComp = polarSpeeds.dTheta
            val chassisComp = Drivetrain.chassisSpeeds.omegaRadiansPerSecond.radiansPerSecond
            val setpoint = clampSafePosition(it.positionSetpoint)
            val offsetCorrection = offsetCorrector.calculate((it.position - setpoint).radians).radiansPerSecond
            val targetVelocity = -offsetCorrection - chassisComp * 0.0 - movementComp * 0.0
            val velocityError = it.velocity - targetVelocity
            val voltage = feedforward.calculate(targetVelocity.radiansPerSecond) + it.PID.calculate(velocityError.radiansPerSecond)
            voltage
        }

        if(Game.sim) setupSim(feedforward)
    }

    // angle of the turret from top view
    var fieldRelativeAngle: Angle
        get() = (turret.position + RobotContainer.navigation.heading).k
        set(value) {
            turret.position = value - RobotContainer.navigation.heading
        }

    init {
        defaultCommand = AimTurret
//        log("SeekTurret as default is off until built", logMode = LogMode.WARN)
    }

    /**
     * Makes an angle safe for the electronics to not get tangled
     */
    fun clampSafePosition(angle: Rotation2d): Angle {
        val norm = angle.k.normalized
        val final = if(norm < 180.degrees) norm else (norm - 360.degrees).k
        return final
    }

    /**
     * Allows the turret to know its position relative to everything else
     */
    fun zeroTurret() {
        turret.resetPosition()
    }

    private val latestResult: PhotonPipelineResult?
        get() = RobotContainer.limelight.latestResult
    val targetVisible: Boolean 
        get() = Game.sim || (latestResult != null && latestResult!!.hasTargets())
    private val target: PhotonTrackedTarget?
        get() = if(targetVisible) latestResult!!.bestTarget else null

    val visionOffset: Angle?
        get() = if (Game.real) {
//            println(target)
            target?.yaw?.let { visionFilter.calculate(it).degrees }
        } else (RobotContainer.navigation.position.towards(Constants.HUB_POSITION) - fieldRelativeAngle).k
    val visionPitch: Angle?
        get() = target?.pitch?.degrees

    val readyToShoot
        get() = turret.positionError < Constants.TURRET_TOLERANCE

    override fun periodic() {
        debugDashboard()
    }

    override fun simulationPeriodic() {
        KField2d.getObject("turret").pose = Pose2d(RobotContainer.navigation.position, fieldRelativeAngle)
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "turret" to turret,
            "turret error" to visionOffset?.radians,
            "field Heading" to fieldRelativeAngle.radians,
            "target detected" to targetVisible
        )
    }
}
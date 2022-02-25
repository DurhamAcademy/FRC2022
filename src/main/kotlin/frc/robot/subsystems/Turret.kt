package frc.robot.subsystems

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.command.LogMode
import frc.kyberlib.math.units.extensions.Angle
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.k
import frc.kyberlib.math.units.extensions.radiansPerSecond
import frc.kyberlib.math.units.towards
import frc.kyberlib.motorcontrol.KSimulatedESC
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.simulation.field.KField2d
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.turret.SeekTurret
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.atan


/**
 * Status of what the turret is doing
 */
enum class TURRET_STATUS {
    LOCKED, ADJUSTING, NOT_FOUND, LOST
}


/**
 * Controls the turret
 */
object Turret : SubsystemBase(), Debug {
    init {
        println("Turret")
    }
    var status = TURRET_STATUS.LOST

    // characterization of the turret
    private val feedforward = SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA)
    // actual turret motors
    val turret = KSparkMax(30).apply {
        kP = 0.1
        kD = .0
        gearRatio = Constants.TURRET_GEAR_RATIO
        motorType = DCMotor.getNeo550(1)

        // fancy control
        val offsetCorrector = ProfiledPIDController(30.00, 0.0, 5.0, TrapezoidProfile.Constraints(12.0, 4.0)).apply { 
            setTolerance(Constants.TURRET_DEADBAND.radians)
            setIntegratorRange(-1.0, 1.0)
         }
        customControl = {
            val polarSpeeds = Drivetrain.polarSpeeds
            val movementComp = polarSpeeds.dTheta
            val chassisComp = Drivetrain.chassisSpeeds.omegaRadiansPerSecond.radiansPerSecond
            val setpoint = clampSafePosition(it.positionSetpoint)
            val offsetCorrection = offsetCorrector.calculate((it.position - setpoint).radians).radiansPerSecond
            val targetVelocity = offsetCorrection - chassisComp - movementComp
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
//        defaultCommand = SeekTurret
        log("SeekTurret as default is off until built", logMode = LogMode.WARN)
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
        get() = if (Game.real) target?.yaw?.degrees
                else (RobotContainer.navigation.position.towards(Constants.HUB_POSITION) - fieldRelativeAngle).k
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
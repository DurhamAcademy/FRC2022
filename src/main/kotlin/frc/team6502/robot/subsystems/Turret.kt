package frc.team6502.robot.subsystems

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.LinearSystemSim
import edu.wpi.first.wpilibj.simulation.SimDeviceSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team6502.robot.Constants
import frc.team6502.robot.RobotContainer
import frc.team6502.robot.commands.drive.Drive
import frc.team6502.robot.commands.turret.SeekTurret
import kyberlib.command.Debug
import kyberlib.command.Game
import kyberlib.math.units.extensions.*
import kyberlib.math.units.towards
import kyberlib.motorcontrol.rev.KSparkMax
import kyberlib.simulation.Simulatable
import kyberlib.simulation.field.KField2d
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.atan


enum class TURRET_STATUS {
    LOCKED, ADJUSTING, NOT_FOUND, LOST
}


object Turret : SubsystemBase(), Debug, Simulatable {
    var status = TURRET_STATUS.LOST

    val feedforward = SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA)
    val turret = KSparkMax(0).apply {
        // todo: tune
        kP = 3.0
        kD = 1.0
//        kI = 10.0
//        kD = 1.0
        addFeedforward(feedforward)
        gearRatio = Constants.TURRET_GEAR_RATIO
    }

    var fieldRelativeAngle: Angle
        get() = (turret.position + RobotContainer.navigation.heading).k
        set(value) {
            turret.position = clampSafePosition(value - RobotContainer.navigation.heading)
        }

    init {
        defaultCommand = SeekTurret
    }

    // todo: check range
    fun clampSafePosition(angle: Rotation2d): Angle = angle.k.normalized

    fun zeroTurret() {
        turret.resetPosition()
    }

    private val latestResult: PhotonPipelineResult?
        get() = RobotContainer.limelight.latestResult
    val targetLost: Boolean
        get() = Game.real && (latestResult == null || !latestResult!!.hasTargets())
    private val target: PhotonTrackedTarget?
        get() = if(!targetLost) latestResult!!.bestTarget else null

    val visionOffset: Angle?
        get() = if (Game.real) target?.yaw?.degrees
                else (RobotContainer.navigation.position.towards(Constants.HUB_POSITION) - fieldRelativeAngle).k
    val visionPitch: Angle?
        get() = target?.pitch?.degrees

    val readyToShoot
        get() = visionOffset != null && visionOffset!!.value < Constants.TURRET_TOLERANCE.value

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
            "target detected" to targetLost
        )
    }

    fun guessVelocity(v: Double): AngularVelocity = ((v - feedforward.ks) / feedforward.kv).radiansPerSecond

    override fun simUpdate(dt: Double) {
        turret.simVelocity = guessVelocity(turret.voltage)
        turret.simPosition = (turret.simPosition + (turret.velocity * dt.seconds)).k
    }
}
package frc.team6502.robot.subsystems

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.LinearSystemSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team6502.robot.Constants
import frc.team6502.robot.RobotContainer
import kyberlib.command.Debug
import kyberlib.command.Game
import kyberlib.math.units.extensions.*
import kyberlib.math.units.towards
import kyberlib.motorcontrol.rev.KSparkMax
import kyberlib.simulation.Simulatable
import kyberlib.simulation.field.KField2d
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget


enum class TURRET_STATUS {
    LOCKED, ADJUSTING, NOT_FOUND, LOST
}


object Turret : SubsystemBase(), Debug, Simulatable {
    var status = TURRET_STATUS.LOST

    private val feedforward = SimpleMotorFeedforward(0.0, 1.0, 0.5)
    val turret = KSparkMax(0).apply {
        // todo: tune
        kP = 3.0
        kD = 1.0
        addFeedforward(feedforward)
        gearRatio = Constants.TURRET_GEAR_RATIO
    }

    var fieldRelativeAngle: Angle
        get() = (turret.position + RobotContainer.navigation.heading).k
        set(value) {
            turret.position = clampSafePosition(value - RobotContainer.navigation.heading)
        }

    // todo: check range
    fun clampSafePosition(angle: Rotation2d): Angle = angle.k.normalized

    fun zeroTurret() {
        turret.resetPosition()
    }

    val latestResult: PhotonPipelineResult?
        get() = RobotContainer.limelight.latestResult
    val targetLost: Boolean
        get() = latestResult == null || !latestResult!!.hasTargets()
    val target: PhotonTrackedTarget?
        get() = if(!targetLost) latestResult!!.bestTarget else null

    val visionOffset: Angle?
        get() = if (Game.real) target?.yaw?.degrees
                else (RobotContainer.navigation.position.towards(Constants.HUB_POSITION) - fieldRelativeAngle).k

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
            "turret error" to visionOffset,
            "target detected" to targetLost
        )
    }

    override fun simUpdate(dt: Double) {
        val velocity = (turret.voltage - feedforward.ks) / feedforward.kv
        turret.simPosition = (turret.simPosition + (velocity * dt).radians).k
    }
}
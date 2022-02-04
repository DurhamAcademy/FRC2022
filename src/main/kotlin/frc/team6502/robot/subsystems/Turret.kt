package frc.team6502.robot.subsystems

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team6502.robot.Constants
import frc.team6502.robot.RobotContainer
import kyberlib.command.Debug
import kyberlib.math.units.extensions.Angle
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.k
import kyberlib.motorcontrol.rev.KSparkMax

object Turret : SubsystemBase(), Debug {
    val turret = KSparkMax(0).apply {
        // todo tune and add ratios
        kP = 30.0
        kD = 5.0

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

    val latestResult
        get() = RobotContainer.limelight.latestResult
    val targetLost
        get() = latestResult == null
    val target
        get() = if(latestResult != null && latestResult.hasTargets()) latestResult.bestTarget else null

    val visionOffset: Angle?
        get() = target?.yaw?.degrees

    var readyToShoot = false

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "turret" to turret,
            "turret error" to visionOffset,
            "target detected" to targetLost
        )
    }
}
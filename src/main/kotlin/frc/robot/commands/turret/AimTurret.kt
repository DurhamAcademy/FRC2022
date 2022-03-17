package frc.robot.commands.turret

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.math.units.extensions.degrees
import frc.robot.subsystems.Turret

/**
 * Keeps the turret aimed at the target
 */
object AimTurret : CommandBase() {
    init {
        addRequirements(Turret)
    }

    override fun initialize() {
        Turret.reset()
    }

    override fun execute() {
        Debug.log("Aim", "execute", level = DebugFilter.Low)

//         if the limelight is a target
        if (Turret.targetVisible) {
//             perp zoom correction
//            val perpSpeed = Drivetrain.polarSpeeds.dTheta.toTangentialVelocity(Drivetrain.polarCoordinates.r)
            var goalOrientation = Turret.visionOffset ?: return
            if (goalOrientation.absoluteValue < 1.degrees) goalOrientation = 0.degrees
            Turret.turret.position = Turret.turret.position + goalOrientation * 0.7
        } else {
            Turret.turret.position = Turret.fieldRelativeAngle
        }
    }

    override fun end(interrupted: Boolean) {
//        Turret.turret.stop()
    }

    /**
     * If you don't find the target after awhile go back to seek turret (looking everywhere).
     */
    override fun isFinished(): Boolean = Turret.lost
}
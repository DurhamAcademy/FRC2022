package frc.robot.commands.turret

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.k
import frc.kyberlib.math.units.extensions.sin
import frc.kyberlib.math.units.towards
import frc.robot.RobotContainer
import frc.robot.subsystems.Shooter
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

        // if the limelight is a target
        if (Turret.targetVisible) {
            val curveCorrection = (RobotContainer.op.curveComp * Turret.turret.position.sin).degrees
            val visionOffset = Turret.visionOffset!! * 0.7
            if(RobotContainer.op.shootWhileMoving) {
                Turret.fieldRelativeAngle = Turret.turret.position + visionOffset + Shooter.movementAngleOffset + curveCorrection
            } else {
                Turret.turret.position = Turret.turret.position + visionOffset + curveCorrection
            }
        } else {
            Turret.turret.position = Turret.turret.position
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
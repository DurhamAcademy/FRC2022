package frc.robot.commands.turret

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.sin
import frc.robot.RobotContainer
import frc.robot.subsystems.Limelight
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Turret

/**
 * Keeps the turret aimed at the target
 */
object AimTurret : CommandBase() {
    init {
        addRequirements(Turret)
    }

    override fun execute() {
        Debug.log("Aim", "execute", level = DebugFilter.Low)

        // if the limelight is a target
        if (Limelight.targetVisible) {
            val curveCorrection = (RobotContainer.op.curveComp * Turret.position.sin).degrees
            val visionOffset = Limelight.visionOffset!!
            val correction = visionOffset + Limelight.movementAngleOffset + curveCorrection
            Turret.position = Turret.position + correction * 0.7
        } else {
            Turret.update()
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
package frc.robot.commands.turret

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.math.units.extensions.k
import frc.kyberlib.math.units.towards
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.subsystems.Limelight
import frc.robot.subsystems.Turret

/**
 * Spin turret in circle. This command should never really be necessary if we odometry good
 */
object SeekTurret : CommandBase() {
    init { addRequirements(Turret) }

    override fun execute() {
        // point towards where hub should be
        Turret.fieldRelativeAngle = RobotContainer.navigation.position.towards(Constants.HUB_POSITION).k
        // if you are sure you see the target
        if (Limelight.targetVisible) {
            // lock onto the target
            AimTurret.schedule()
        }
    }
}
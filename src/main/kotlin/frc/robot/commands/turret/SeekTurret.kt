package frc.robot.commands.turret

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.robot.subsystems.TurretStatus
import frc.robot.subsystems.Turret
import frc.kyberlib.math.units.extensions.k
import frc.kyberlib.math.units.extensions.rotations
import frc.kyberlib.math.units.towards
import frc.robot.Constants
import frc.robot.RobotContainer

/**
 * Spin turret in circle. This command should never really be necessary if we odometry good
 */
object SeekTurret : CommandBase() {
    init {
        addRequirements(Turret)
    }

    /**
     * Update status to indicated turret is lost
     */
    override fun initialize() {
        Turret.reset()
        Turret.status = TurretStatus.LOST
    }

    override fun execute() {
        Debug.log("Seek", "execute", level= DebugFilter.Low)
        // sweeps menacingly - test this
        Turret.fieldRelativeAngle = RobotContainer.navigation.position.towards(Constants.HUB_POSITION).k

        // if you are sure you see the target
        if(!Turret.lost) {
            // lock onto the target
            AimTurret.schedule()
        }
    }

    override fun end(interrupted: Boolean) {
        if (!interrupted) Turret.status = TurretStatus.ADJUSTING
        Turret.turret.stop()
    }
}
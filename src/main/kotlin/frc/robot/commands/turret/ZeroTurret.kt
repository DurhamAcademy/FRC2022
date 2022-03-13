package frc.robot.commands.turret

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.subsystems.Turret
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.command.Game

/**
 * Called at the beginning of the match to get the absolute encoding of the turret
 */
object ZeroTurret: CommandBase() {
    init {
        addRequirements(Turret)
    }

    override fun execute() {
        Turret.turret.percent = -0.1  // TODO: check this direction
        Debug.log("turretLimit", RobotContainer.turretLimit.get().toString(), level = DebugFilter.Low)
    }

    override fun end(interrupted: Boolean) {
        Debug.log("turretLimit", "zeroed")
        Turret.zeroTurret()
        Turret.turret.stop()
    }

    override fun isFinished() = !RobotContainer.turretLimit.get()

}
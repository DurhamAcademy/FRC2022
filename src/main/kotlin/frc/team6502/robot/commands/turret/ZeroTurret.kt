package frc.team6502.robot.commands.turret

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.RobotContainer
import frc.team6502.robot.subsystems.Turret
import kyberlib.command.Debug
import kyberlib.command.DebugLevel

/**
 * Called at the beginning of the match to get the absolute encoding of the turret
 */
object ZeroTurret: CommandBase() {
    init {
        addRequirements(Turret)
    }

    override fun execute() {
        Turret.turret.percent = 0.1  // todo: check this direction
        Debug.log("turretLimit", RobotContainer.turretLimit.get().toString(), level = DebugLevel.LowPriority)
    }

    override fun end(interrupted: Boolean) {
        Debug.log("turretLimit", "zeroed")
        Turret.zeroTurret()
    }

    override fun isFinished() = !RobotContainer.turretLimit.get()

}
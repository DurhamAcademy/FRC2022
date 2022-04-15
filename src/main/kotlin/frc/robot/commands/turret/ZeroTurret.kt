package frc.robot.commands.turret

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.command.Game
import frc.robot.RobotContainer
import frc.robot.subsystems.Turret

/**
 * Called at the beginning of the match to get the absolute encoding of the turret
 */
object ZeroTurret : CommandBase() {

    private val timer = Timer()

    init {
        addRequirements(Turret)
    }

    override fun initialize() {
        Turret.isZeroed = false
        timer.start()
    }

    override fun execute() {
        if (timer.hasElapsed(0.3)) Turret.turret.percent = -0.2
        else Turret.turret.percent = 0.5
    }

    override fun end(interrupted: Boolean) {
        Turret.zeroTurret()
    }

    override fun isFinished() = !RobotContainer.turretLimit.get() || Game.sim
}

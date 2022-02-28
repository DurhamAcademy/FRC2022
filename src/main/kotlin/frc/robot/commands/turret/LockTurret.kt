package frc.robot.commands.turret

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.TURRET_STATUS
import frc.robot.subsystems.Turret

object LockTurret : CommandBase() {
    init {
        addRequirements(Turret)
    }

    override fun initialize() {
        Turret.turret.stop()
        Turret.status = TURRET_STATUS.FROZEN
    }
}
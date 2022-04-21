package frc.robot.commands.turret

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.TurretStatus
import frc.robot.subsystems.Turret

object FreezeTurret : CommandBase() {
    init {
        addRequirements(Turret)
    }

    override fun initialize() {
        Turret.stop()
        Turret.status = TurretStatus.FROZEN
    }
}
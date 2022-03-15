package frc.robot.commands.turret

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.degrees
import frc.robot.subsystems.Turret
import frc.robot.subsystems.TurretStatus

object LostTurret : CommandBase() {
    init {
        addRequirements(Turret)
    }

    override fun initialize() {
        Turret.status = TurretStatus.LOST
    }

    private var direction = 1.0
    override fun execute() {
        Turret.turret.percent = 0.1 * direction
        if(Turret.turret.position < Turret.turret.minPosition + 5.degrees) direction = 1.0
        else if(Turret.turret.position > Turret.turret.maxPosition - 5.degrees) direction = -1.0
    }

    override fun end(interrupted: Boolean) {
        AimTurret.schedule()
    }

    override fun isFinished(): Boolean = !Turret.lost
}
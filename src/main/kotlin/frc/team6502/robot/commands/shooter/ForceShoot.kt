package frc.team6502.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.subsystems.Conveyor
import frc.team6502.robot.subsystems.Shooter
import frc.team6502.robot.subsystems.Turret

object ForceShoot : CommandBase() {
    init {
        addRequirements(Conveyor, Turret, Shooter)
    }

    override fun execute() {
        Shooter.flywheelMaster.percent = 0.5
        Shooter.topShooter.percent = 0.5
    }
}
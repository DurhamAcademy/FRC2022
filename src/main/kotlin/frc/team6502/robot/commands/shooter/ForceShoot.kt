package frc.team6502.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.subsystems.Conveyor
import frc.team6502.robot.subsystems.SHOOTER_STATUS
import frc.team6502.robot.subsystems.Shooter
import frc.team6502.robot.subsystems.Turret

/**
 * Bypass the turret restrictions and forces the robot to shoot
 */
object ForceShoot : CommandBase() {
    init {
        addRequirements(Conveyor, Turret, Shooter)
    }

    override fun initialize() {
        Shooter.status = SHOOTER_STATUS.FORCE_SHOT
    }

    override fun execute() {
        Shooter.flywheelMaster.percent = 0.5
        Shooter.topShooter.percent = 0.5
        Conveyor.feed()
    }

}
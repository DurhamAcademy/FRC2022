package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.SHOOTER_STATUS
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Turret
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugLevel

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
        Debug.log("Force Shoot", "execute", level=DebugLevel.LowPriority)
        Shooter.flywheelMaster.percent = 0.5
        Shooter.topShooter.percent = 0.5
        Conveyor.feed()
    }

}
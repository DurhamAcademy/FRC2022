package frc.robot.commands.shooter

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Game
import frc.kyberlib.pneumatics.KSolenoid
import frc.robot.RobotContainer
import frc.robot.subsystems.*

class AutoShot : CommandBase() {
    init {
        addRequirements(Shooter, Drivetrain, Conveyor)
    }

    override fun initialize() {
        Intaker.stop()
        Intaker.deployed = false
        KSolenoid.compressor.disable()
        shootingTimer.reset()
        Drivetrain.stop()
    }

    val shootingTimer = Timer()

    override fun execute() {
        Shooter.update()
        if (Turret.ready && Shooter.ready) {
            shootingTimer.start()
            Shooter.status = ShooterStatus.SHOT
            Conveyor.feed()
        } else {
            Shooter.status = ShooterStatus.SPINUP
            Conveyor.prepare()
        }
    }

    override fun end(interrupted: Boolean) {
        if (RobotContainer.op.compressor) KSolenoid.compressor.enableDigital()
        Shooter.stop()
        if (Game.AUTO) {
            Conveyor.autoManage()
        } else {
            Intaker.deployed = false
            Intaker.stop()
        }
    }

    override fun isFinished(): Boolean = shootingTimer.hasElapsed(2.0)
}
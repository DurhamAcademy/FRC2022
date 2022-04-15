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
        Intaker.intakeMotor.stop()
        Intaker.deployed = false
        shootingTimer.reset()
        Drivetrain.stop()
    }

    val shootingTimer = Timer()

    override fun execute() {
        Shooter.update()
        if (Turret.ready && Shooter.ready) {
            shootingTimer.start()
            Conveyor.feed()
        } else {
            Conveyor.prepare()
        }
    }

    override fun end(interrupted: Boolean) {
        Shooter.stop()
        if (Game.AUTO) {
            Conveyor.conveyor.percent = 0.1
            Conveyor.feeder.percent = -0.1
        } else {
            Intaker.deployed = false
            Intaker.intakeMotor.stop()
        }
    }

    override fun isFinished(): Boolean = shootingTimer.hasElapsed(2.0)
}
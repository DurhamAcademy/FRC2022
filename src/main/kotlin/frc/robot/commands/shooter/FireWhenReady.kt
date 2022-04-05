package frc.robot.commands.shooter

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.subsystems.*

object FireWhenReady : CommandBase() {
    init {
        addRequirements(Shooter, Conveyor)
    }

    val feedTimer = Timer()

    override fun initialize() {
        feedTimer.reset()
        feedTimer.stop()
    }

    override fun execute() {
        val hopperStatus = Conveyor.status
        when(Conveyor.status) {
            ConveyorStatus.SINGLE_GOOD -> {
                Shooter.update()
                if ((Turret.targetVisible || Shooter.status == ShooterStatus.SHOT)) {
//            val parallelSpeed = Drivetrain.polarSpeeds.dr
                    Shooter.update()

                    // if the turret is on target
                    if (Turret.ready && Shooter.ready) {
                        Shooter.status = ShooterStatus.SHOT
                        Conveyor.status = ConveyorStatus.FEEDING
                        Conveyor.feed()
                        feedTimer.start()
                    } else {
                        Shooter.status = ShooterStatus.SPINUP
                        Conveyor.prepare()
                    }
                }
            }
            ConveyorStatus.BAD -> {
                Dispose.schedule()
            }
            ConveyorStatus.FEEDING -> {
                Shooter.update()
                if(feedTimer.hasElapsed(1.5)) Conveyor.idle()
            }
            else -> {
                Shooter.stop()
                Conveyor.idle()
            }
        }
    }
}
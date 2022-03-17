package frc.robot.commands.shooter

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.pneumatics.KSolenoid
import frc.robot.subsystems.*

class AutoShot : CommandBase() {
    init {
        addRequirements(Shooter, Drivetrain, Conveyor)
    }

    private var reenableCompressor = true
    override fun initialize() {
        reenableCompressor = KSolenoid.compressor.enabled()
        KSolenoid.compressor.disable()
        shootingTimer.reset()
        Drivetrain.stop()
    }

    val shootingTimer = Timer()

    override fun execute() {
        if ((Turret.targetVisible || Shooter.status == ShooterStatus.SHOT)) {
            Shooter.update()

            // if the turret is on target
            if (Turret.ready && Shooter.ready) {
                shootingTimer.start()
                Shooter.status = ShooterStatus.SHOT
                Conveyor.feed()
            } else {
                Shooter.status = ShooterStatus.SPINUP
                Conveyor.stop()
            }
        }
    }

    override fun end(interrupted: Boolean) {
        if (reenableCompressor) KSolenoid.compressor.enableDigital()
    }

    override fun isFinished(): Boolean = shootingTimer.hasElapsed(2.0)
}
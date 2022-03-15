package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.pneumatics.KSolenoid
import frc.robot.subsystems.*

/**
 * Bypass the turret restrictions and forces the robot to shoot
 */
object ForceShoot : CommandBase() {
    init {
        addRequirements(Shooter, Conveyor)
    }

    private var reenableCompressor = true
    override fun initialize() {
        reenableCompressor = KSolenoid.compressor.enabled()
        KSolenoid.compressor.disable()
        Conveyor.feed()
        Shooter.status = ShooterStatus.FORCE_SHOT
    }

    override fun execute() {
        Debug.log("Force Shoot", "execute", level=DebugFilter.Low)

        if (Shooter.ready) {
            Shooter.update()
        }
        else {
            Shooter.flywheel.percent = 0.5
        }
    }

    override fun end(interrupted: Boolean) {
        Shooter.stop()
        Conveyor.stop()
        if(reenableCompressor) KSolenoid.compressor.enableDigital()
    }

}
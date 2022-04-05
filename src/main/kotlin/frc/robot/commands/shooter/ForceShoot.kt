package frc.robot.commands.shooter

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.pneumatics.KSolenoid
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.ShooterStatus

/**
 * Bypass the turret restrictions and forces the robot to shoot
 */
object ForceShoot : CommandBase() {
    init {
        addRequirements(Shooter, Conveyor)
        SmartDashboard.putNumber("zoom", 0.8)
    }

    private var reenableCompressor = true
    override fun initialize() {
        reenableCompressor = KSolenoid.compressor.enabled()
        KSolenoid.compressor.disable()
        Conveyor.feed()
        Shooter.status = ShooterStatus.FORCE_SHOT
    }

    override fun execute() {
        Debug.log("Force Shoot", "execute", level = DebugFilter.Low)

        if (true || Shooter.ready) {
            Shooter.update()
        } else {
            Shooter.flywheel.percent = SmartDashboard.getNumber("zoom", 0.5)//0.5
        }
    }

    override fun end(interrupted: Boolean) {
        Shooter.stop()
        Conveyor.stop()
        if (reenableCompressor) KSolenoid.compressor.enableDigital()
    }

}
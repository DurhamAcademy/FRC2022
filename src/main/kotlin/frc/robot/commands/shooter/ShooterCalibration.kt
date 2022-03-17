package frc.robot.commands.shooter

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.pneumatics.KSolenoid
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.Shooter

object ShooterCalibration : CommandBase() {

    const val rpmString = "flywheel rpm"
    const val hoodString = "hood degrees"
    init {
        addRequirements(Shooter, Conveyor)
        SmartDashboard.putNumber(rpmString, 0.0)
        SmartDashboard.putNumber(hoodString, 5.0)
    }

    private var reenableCompressor = true
    override fun initialize() {
        reenableCompressor = KSolenoid.compressor.enabled()
        KSolenoid.compressor.disable()
    }

    override fun execute() {
        // 0 - 3000 rpm limits
//        Shooter.update()
        Shooter.targetVelocity = SmartDashboard.getNumber(rpmString, 0.0).rpm
        Shooter.hoodDistance = SmartDashboard.getNumber(hoodString, 0.0).millimeters

        if(Shooter.ready) {
//            Feed.schedule()
            Conveyor.feed()
        }

    }

    override fun end(interrupted: Boolean) {
        Shooter.stop()
        Conveyor.stop()
        if(reenableCompressor) KSolenoid.compressor.enableDigital()
    }

    override fun isFinished(): Boolean = false
}
package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.pneumatics.KSolenoid
import frc.robot.RobotContainer
import frc.robot.subsystems.*


/**
 * Default Shooter method to automatically shoot balls when ready
 */
object Shoot : CommandBase() {
    init {
        addRequirements(Shooter, Conveyor)
    }

    private var reenableCompressor = true
    override fun initialize() {
        reenableCompressor = KSolenoid.compressor.enabled()
        KSolenoid.compressor.disable()
    }

    override fun execute() {
        Debug.log("Shoot", "execute", level = DebugFilter.Low)
        // check if shooter should spin up
        if ((Turret.targetVisible || Shooter.status == ShooterStatus.SHOT)) {
//            val parallelSpeed = Drivetrain.polarSpeeds.dr
            Shooter.update()

            // if the turret is on target
            if (Turret.ready && Shooter.ready) {
                Shooter.status = ShooterStatus.SHOT
                Conveyor.status = ConveyorStatus.FEEDING
                Conveyor.feed()
                RobotContainer.controller.rumble = 0.0
            } else {
                RobotContainer.controller.rumble = 0.5
                Shooter.status = ShooterStatus.SPINUP
                Conveyor.prepare()
            }
        }
    }

    override fun end(interrupted: Boolean) {
        Debug.log("Shoot", "idle", level = DebugFilter.Low)
        Shooter.stop()
        Conveyor.stop()
        RobotContainer.controller.rumble = 0.0
        if (reenableCompressor || true) KSolenoid.compressor.enableDigital()

    }
}
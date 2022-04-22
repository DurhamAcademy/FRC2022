package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.pneumatics.KSolenoid
import frc.robot.RobotContainer
import frc.robot.commands.drive.AimDrive
import frc.robot.subsystems.*


/**
 * Default Shooter method to automatically shoot balls when ready
 */
class Shoot : CommandBase() {
    init {
        addRequirements(Shooter, Conveyor)
    }

    override fun initialize() {
        AimDrive.schedule()
    }

    override fun execute() {
        Debug.log("Shoot", "execute", level = DebugFilter.Low)
        // check if shooter should spin up
        Shooter.update()
        // if the turret is on target
        if (Limelight.targetVisible && Limelight.visionOffset.absoluteValue < 3.degrees && Shooter.ready) {
            Shooter.status = ShooterStatus.SHOT
            Conveyor.feed()
            RobotContainer.controller.rumble = 0.0
        } else {
            RobotContainer.controller.rumble = 0.5
            Shooter.status = ShooterStatus.SPINUP
            Conveyor.prepare()
        }
    }

    override fun end(interrupted: Boolean) {
        Debug.log("Shoot", "idle", level = DebugFilter.Low)
        Shooter.stop()
        Conveyor.stop()
        AimDrive.cancel()
        RobotContainer.controller.rumble = 0.0

    }
}
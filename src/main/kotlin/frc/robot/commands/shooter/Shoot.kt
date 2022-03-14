package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.math.units.extensions.*
import frc.robot.RobotContainer
import frc.robot.commands.intake.Feed
import frc.robot.commands.intake.Idle
import frc.robot.subsystems.*


/**
 * Default Shooter method to automatically shoot balls when ready
 */
object Shoot : CommandBase() {
    init {
        addRequirements(Shooter)
    }
    override fun execute() {
        Debug.log("Shoot", "execute", level=DebugFilter.Low)
        // check if shooter should spin up
        if ((Turret.targetVisible || Shooter.status == ShooterStatus.SHOT)) {
//            val parallelSpeed = Drivetrain.polarSpeeds.dr
            // calculate values given the current distance from the hub
            // top rpm: 5676 RPM
            // top theorectical out velocity = 144.1704 m/s
            // top useful out velocity = 12.7 m/s
            // min useful = 6.4516 m/s
            // tof(angle) =
            Shooter.update()

            // if the turret is on target
            if (Turret.ready && Shooter.ready) {
                Shooter.status = ShooterStatus.SHOT
                Feed.schedule()
            } 
            else {
                RobotContainer.controller.rumble = 0.5
                Shooter.status = ShooterStatus.SPINUP
            }
        }
    }

    override fun end(interrupted: Boolean) {
        Debug.log("Shoot", "idle", level=DebugFilter.Low)
        Shooter.stop()
    }
}
package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.pneumatics.KSolenoid
import frc.robot.RobotContainer
import frc.robot.commands.intake.Feed
import frc.robot.commands.intake.Idle
import frc.robot.subsystems.*


/**
 * Default Shooter method to automatically shoot balls when ready
 */
object Shoot : CommandBase() {
    init {
        addRequirements(Shooter, Conveyor)
    }

    override fun initialize() {
        KSolenoid.compressor.disable()
    }
    override fun execute() {
        Debug.log("Shoot", "execute", level=DebugFilter.Low)
        // check if shooter should spin up
        if ((Turret.targetVisible || Shooter.status == ShooterStatus.SHOT)) {
//            val parallelSpeed = Drivetrain.polarSpeeds.dr
            Shooter.update()

            // if the turret is on target
            if (Turret.ready && Shooter.ready) {
                Shooter.status = ShooterStatus.SHOT
//                Feed.schedule()
                Conveyor.status = ConveyorStatus.FEEDING
                Conveyor.conveyor.percent = 0.8
                Conveyor.feeder.percent = 0.9
                RobotContainer.controller.rumble = 0.0
            } 
            else {
                RobotContainer.controller.rumble = 0.5
                Shooter.status = ShooterStatus.SPINUP
//                Conveyor.conveyor.percent = -.1
            }
        }
    }

    override fun end(interrupted: Boolean) {
        Debug.log("Shoot", "idle", level=DebugFilter.Low)
        Shooter.stop()
        Idle.initialize()
        RobotContainer.controller.rumble = 0.0
        KSolenoid.compressor.enableDigital()

    }
}
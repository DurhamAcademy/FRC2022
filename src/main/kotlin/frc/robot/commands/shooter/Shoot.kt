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

    var prevDistance = 1.0

    override fun execute() {
        Debug.log("Shoot", "execute", level=DebugFilter.Low)
        // check if shooter should spin up
        if ((Turret.targetVisible || Shooter.status == ShooterStatus.SHOT)) {
            val dis = if (Turret.targetVisible) Shooter.targetDistance!!.meters else prevDistance
//            val parallelSpeed = Drivetrain.polarSpeeds.dr
            prevDistance = dis

            // calculate values given the current distance from the hub
            // top rpm: 5676 RPM
            // top theorectical out velocity = 144.1704 m/s
            // top useful out velocity = 12.7 m/s
            // min useful = 6.4516 m/s
            // tof(angle) =
            val targetFlywheelVelocity = Constants.FLYWHEEL_INTERPOLATOR.calculate(dis)!!.radiansPerSecond
            val targetTopWheelVelocity = targetFlywheelVelocity + 50.rpm//Constants.TOPWHEEL_INTERPOLATOR.calculate(dis)!!.radiansPerSecond
            val targetHoodAngle = Constants.HOODANGLE_INTERPOLATOR.calculate(dis)!!

            // set the positions/velocities to the motors
            Shooter.flywheelMaster.velocity = targetFlywheelVelocity
            Shooter.hoodDistance = (targetHoodAngle / 10).centimeters

            // if the turret is on target
            if (Turret.readyToShoot && Shooter.flywheelMaster.velocityError < Constants.SHOOTER_VELOCITY_TOLERANCE && Shooter.hood.atSetpoint) {
                Shooter.status = ShooterStatus.SHOT
                Feed.schedule()
            } 
            else {
                Idle.execute()
                RobotContainer.controller.rumble = 0.5
                Shooter.status = ShooterStatus.SPINUP
            }
        }
    }

    override fun end(interrupted: Boolean) {
        Debug.log("Shoot", "idle", level=DebugFilter.Low)
        Shooter.flywheelMaster.stop()
        Shooter.status = ShooterStatus.IDLE
    }
}
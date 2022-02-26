package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.math.units.extensions.radiansPerSecond
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugLevel
import frc.robot.subsystems.*


/**
 * Default Shooter method to automatically shoot balls when ready
 */
object Shoot : CommandBase() {
    init {
        addRequirements(Conveyor, Shooter)
    }

    var prevDistance = 1.0

    override fun execute() {
        Debug.log("Shoot", "execute", level=DebugLevel.LowPriority)
        // check if shooter should spin up
        if (Conveyor.good && (Turret.targetVisible || Conveyor.status == CONVEYOR_STATUS.FEEDING)) {
            val dis = if (Turret.targetVisible) Shooter.targetDistance!!.meters else prevDistance
            val parallelSpeed = Drivetrain.polarSpeeds.dr
            prevDistance = dis

            // calculate values given the current distance from the hub
            val targetFlywheelVelocity = Constants.FLYWHEEL_INTERPOLATOR.calculate(dis)!!.radiansPerSecond
            val targetTopWheelVelocity = Constants.TOPWHEEL_INTERPOLATOR.calculate(dis)!!.radiansPerSecond
            val targetHoodAngle = Constants.HOODANGLE_INTERPOLATOR.calculate(dis)!!.degrees

            // set the positions/velocities to the motors
            Shooter.flywheelMaster.velocity = targetFlywheelVelocity
            Shooter.topShooter.velocity = targetTopWheelVelocity
            Shooter.hoodAngle = targetHoodAngle

            // if the turret is on target
            if (Turret.readyToShoot && Shooter.flywheelMaster.velocityError < Constants.SHOOTER_VELOCITY_TOLERANCE) {
                Conveyor.feed()
                Shooter.status = SHOOTER_STATUS.AUTO_SHOT
            } 
            else Shooter.status = SHOOTER_STATUS.SPINUP
        } 
        else {
            Debug.log("Shoot", "idle", level=DebugLevel.LowPriority)
            Shooter.flywheelMaster.stop()
            Shooter.topShooter.stop()
            Shooter.status = SHOOTER_STATUS.IDLE
        }
    }

    override fun isFinished(): Boolean = false
}
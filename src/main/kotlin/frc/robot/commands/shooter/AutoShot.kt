package frc.robot.commands.shooter

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.*
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.intake.Feed
import frc.robot.commands.intake.Idle
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.ShooterStatus
import frc.robot.subsystems.Turret

class AutoShot : CommandBase() {
    init {
        addRequirements(Shooter, Drivetrain)
    }

    override fun initialize() {
        Drivetrain.stop()
    }

    val shootingTimer = Timer()

    override fun execute() {
        if ((Turret.targetVisible || Shooter.status == ShooterStatus.SHOT)) {
            val dis = if (Turret.targetVisible) Shooter.targetDistance!!.meters else RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION)
            val targetFlywheelVelocity = Constants.FLYWHEEL_INTERPOLATOR.calculate(dis)!!.radiansPerSecond
            val targetHoodAngle = Constants.HOODANGLE_INTERPOLATOR.calculate(dis)!!

            // set the positions/velocities to the motors
            Shooter.flywheelMaster.velocity = targetFlywheelVelocity
            Shooter.hoodAngle = targetHoodAngle.degrees

            // if the turret is on target
            if (Turret.readyToShoot && Shooter.flywheelMaster.velocityError < Constants.SHOOTER_VELOCITY_TOLERANCE && Shooter.hood.atSetpoint) {
                shootingTimer.start()
                Shooter.status = ShooterStatus.SHOT
                Feed.schedule()
            }
            else {
                Idle.execute()
                Shooter.status = ShooterStatus.SPINUP
            }
        }
    }

    override fun isFinished(): Boolean = shootingTimer.hasElapsed(2.0)
}
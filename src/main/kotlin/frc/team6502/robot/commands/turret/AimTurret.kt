package frc.team6502.robot.commands.turret

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.Constants
import frc.team6502.robot.RobotContainer
import frc.team6502.robot.subsystems.Drivetrain
import frc.team6502.robot.subsystems.Turret
import kyberlib.math.units.extensions.k
import kyberlib.math.units.extensions.metersPerSecond
import kyberlib.math.units.extensions.radiansPerSecond
import kyberlib.math.units.towards
import kotlin.math.sin

object AimTurret : CommandBase() {

    init {
        addRequirements(Turret)
    }

    val notFoundTimer = Timer()  // timer counting how long without vision target seen
    val lostTimer = Timer()  // timer after how long not finding it at the expected location

    // todo tune this
    val offsetCorrector = ProfiledPIDController(30.0, 2.0, 5.0, TrapezoidProfile.Constraints(1.0, 1.0))

    override fun initialize() {
        notFoundTimer.reset()
        notFoundTimer.stop()
        lostTimer.reset()
        lostTimer.stop()
    }

    override fun execute() {
        // spin stablization
        val chassisRotation = Drivetrain.chassisSpeeds.omegaRadiansPerSecond.radiansPerSecond

        var visionRotation = 0.radiansPerSecond
        if (!Turret.targetLost) {
            notFoundTimer.reset()
            notFoundTimer.stop()
            lostTimer.reset()
            lostTimer.stop()

            // perp zoom correction
//            val towardsHub = Turret.turret.position + Turret.visionOffset
//            val robotSpeed = Drivetrain.chassisSpeeds.vxMetersPerSecond.metersPerSecond
//            val perpSpeed = robotSpeed * sin(towardsHub.radians)

            // todo: add perp control later
            val goalOrientation = Turret.visionOffset
            visionRotation = offsetCorrector.calculate(goalOrientation!!.radians).radiansPerSecond
        } else {
            notFoundTimer.start()
            if (notFoundTimer.hasElapsed(.5)) {
                lostTimer.start()

                Turret.fieldRelativeAngle = RobotContainer.navigation.position.towards(Constants.HUB_POSITION).k
            }
        }

//        val turretVelocity = chassisRotation * -1.0 + visionRotation
//        val targetAngle = Turret.clampSafePosition(Turret.turret.position + goalOrientation) - Turret.turret.position
        Turret.turret.velocity = chassisRotation * -1.0 + visionRotation
        // TODO: FIX - WARNING!!! this is testing only, safe angles are not enabled and something will have a seizure
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            // flash the LEDs something
        }
    }

    override fun isFinished(): Boolean = lostTimer.hasElapsed(2.0)
}
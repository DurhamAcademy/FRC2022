package frc.robot.commands.turret

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.TURRET_STATUS
import frc.robot.subsystems.Turret
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.towards
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugLevel
import kotlin.math.sin

/**
 * Keeps the turret aimed at the target
 */
object AimTurret : CommandBase() {
    init {
        addRequirements(Turret)
    }

    val notFoundTimer = Timer()  // timer counting how long without vision target seen
    val lostTimer = Timer()  // timer after how long not finding it at the expected location

    override fun initialize() {
        found()
    }

    private fun found() {
        notFoundTimer.reset()
        notFoundTimer.stop()
        lostTimer.reset()
        lostTimer.stop()
    }

    override fun execute() {
        Debug.log("Aim", "execute", level=DebugLevel.LowPriority)
        // if the limelight is a target
        if (!Turret.targetLost) {
            found()

            // perp zoom correction todo: add later
//            val towardsHub = Turret.turret.position + Turret.visionOffset
//            val robotSpeed = Drivetrain.chassisSpeeds.vxMetersPerSecond.metersPerSecond
//            val perpSpeed = robotSpeed * sin(towardsHub.radians)

            val goalOrientation = Turret.visionOffset!!
            Turret.fieldRelativeAngle = Turret.fieldRelativeAngle + goalOrientation
        }
        else {
            notFoundTimer.start()
            // wait for awhile to make sure the target is lost
            if (notFoundTimer.hasElapsed(Constants.NOT_FOUND_WAIT)) {
                Turret.status = TURRET_STATUS.NOT_FOUND
                lostTimer.start()

                // look towards where the hub should be
                Turret.fieldRelativeAngle = RobotContainer.navigation.position.towards(Constants.HUB_POSITION).k
            }
        }
    }

    /**
     * If you don't find the target after awhile go back to seek turret (looking everywhere).
     */
    override fun isFinished(): Boolean = lostTimer.hasElapsed(Constants.LOST_WAIT) || (lostTimer.hasElapsed(0.001) && !Constants.SMART_LOSS)
}
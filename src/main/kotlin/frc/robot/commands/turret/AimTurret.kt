package frc.robot.commands.turret

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.math.units.extensions.k
import frc.kyberlib.math.units.towards
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.subsystems.TURRET_STATUS
import frc.robot.subsystems.Turret

/**
 * Keeps the turret aimed at the target
 */
object AimTurret : CommandBase() {
    init {
        addRequirements(Turret)
    }

    private val notFoundTimer = Timer()  // timer counting how long without vision target seen
    private val lostTimer = Timer()  // timer after how long not finding it at the expected location

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
        Debug.log("Aim", "execute", level=DebugFilter.Low)
        // if the limelight is a target
        if (Turret.targetVisible) {
            found()

            // perp zoom correction
//            val perpSpeed = Drivetrain.polarSpeeds.dTheta.toTangentialVelocity(Drivetrain.polarCoordinates.r)

            val goalOrientation = Turret.visionOffset!!
            Turret.fieldRelativeAngle = Turret.fieldRelativeAngle + goalOrientation
        }
        else {
            Turret.turret.stop()
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

    override fun end(interrupted: Boolean) {
        Turret.turret.stop()
    }

    /**
     * If you don't find the target after awhile go back to seek turret (looking everywhere).
     */
    override fun isFinished(): Boolean = lostTimer.hasElapsed(Constants.LOST_WAIT) || (lostTimer.hasElapsed(0.001) && !Constants.SMART_LOSS)
}
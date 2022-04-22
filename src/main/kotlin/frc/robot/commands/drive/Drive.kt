package frc.robot.commands.drive

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.math.units.extensions.feetPerSecond
import frc.kyberlib.math.units.extensions.metersPerSecond
import frc.kyberlib.math.units.extensions.radiansPerSecond
import frc.kyberlib.math.units.extensions.rpm
import frc.robot.RobotContainer
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Shooter

/**
 * The default drive commands. Arcade drive based on xbox controller inputs
 */
object Drive : CommandBase() {

    private val velFilter = SlewRateLimiter(10.0) // mps
    private val rotFilter = SlewRateLimiter(200.0)

    init {
        addRequirements(Drivetrain)
    }

    /**
     * Get xbox inputs and drive corresponding values
     */
    override fun execute() {
        // todo: filter inputs for smoother drive
//        val fwd = velFilter.calculate(RobotContainer.controlScheme.DRIVE_FORWARD).feetPerSecond
//        val turn = rotFilter.calculate(RobotContainer.controlScheme.DRIVE_TURN).radiansPerSecond
        val fwd = RobotContainer.controller.rightY.value.feetPerSecond
        val strafe = RobotContainer.controller.rightX.value.feetPerSecond
        val turn = RobotContainer.controller.leftX.value.radiansPerSecond
        val speeds = ChassisSpeeds(fwd.metersPerSecond, strafe.metersPerSecond, turn.radiansPerSecond)
        Debug.log("Default Drive", "fwd: $fwd, turn: $turn", level = DebugFilter.Low)
        Drivetrain.drive(speeds)
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }

    override fun isFinished() = false
}

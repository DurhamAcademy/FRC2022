package frc.robot.commands.drive

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.subsystems.Drivetrain
import frc.kyberlib.math.units.extensions.feetPerSecond
import frc.kyberlib.math.units.extensions.metersPerSecond
import frc.kyberlib.math.units.extensions.radiansPerSecond
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter

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
        val fwd = velFilter.calculate(RobotContainer.controlScheme.DRIVE_FORWARD).feetPerSecond
        val turn = rotFilter.calculate(RobotContainer.controlScheme.DRIVE_TURN).radiansPerSecond
        val speeds = ChassisSpeeds(velFilter.calculate(fwd.metersPerSecond), 0.0, rotFilter.calculate(turn.radiansPerSecond))
        Debug.log("Default Drive", "fwd: $fwd, turn: $turn", level=DebugFilter.LowPriority)
        Drivetrain.drive(speeds)
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }

    override fun isFinished() = false
}

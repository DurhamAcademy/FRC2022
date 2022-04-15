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
    init {
        addRequirements(Drivetrain)
    }

    /**
     * Get xbox inputs and drive corresponding values
     */
    override fun execute() {
        val fwd = (RobotContainer.controller.rightTrigger.value - RobotContainer.controller.leftTrigger.value).feetPerSecond
        val turn = RobotContainer.controller.rightX.value.radiansPerSecond
        val speeds = ChassisSpeeds(fwd.metersPerSecond, 0.0, turn.radiansPerSecond)
        Debug.log("Default Drive", "fwd: $fwd, turn: $turn", level = DebugFilter.Low)
        Drivetrain.drive(speeds)
    }

    override fun isFinished() = false
}

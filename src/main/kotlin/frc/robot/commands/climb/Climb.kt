package frc.robot.commands.climb

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.inches
import frc.robot.RobotContainer
import frc.robot.commands.drive.Drive
import frc.robot.subsystems.*
import kotlin.math.absoluteValue


/**
 * Command that uses joysticks to control the climber extension.
 */
object Climb : CommandBase() {
    init {
        addRequirements(Climber, Drivetrain, Turret, Shooter)
    }

    /**
     * Prepare the climber to climb.
     */
    override fun initialize() {
        Debug.log("Climb Command", "init", level = DebugFilter.Low)
        Turret.turret.position = 0.degrees
        Climber.staticsLifted = true
        Climber.status = ClimberStatus.ACTIVE
    }

    var hasFallen = false
    /**
     * Set the winch percentage based on left/right joysticks
     */
    override fun execute() {
        Debug.log("Climb Command", "execute", level = DebugFilter.Low)
        Turret.turret.position = 0.degrees

        Climber.leftWinch.percent = RobotContainer.controller.leftY.raw()
//        Climber.leftExtendable.percent = RobotContainer.controller.leftX.raw()
        Climber.rightWinch.percent = RobotContainer.controller.rightY.raw()
//        Climber.rightExtendable.percent = RobotContainer.controller.rightX.raw()


        // set the status of the robot based on what the winches are doing
        if (Climber.leftWinch.percent.absoluteValue < 0.1) Climber.status = ClimberStatus.ACTIVE
        else if (Climber.leftWinch.percent < 0.0) {
            Climber.status = ClimberStatus.FALLING
            hasFallen = true
        }
        else if (Climber.leftWinch.percent.absoluteValue > 0.0) Climber.status = ClimberStatus.RISING

        if (RobotContainer.op.climbStabilization != 0.0 && Climber.leftWinch.linearPosition > 5.inches && hasFallen)
            Climber.stabalize()
        else Drive.execute()
    }

    override fun isFinished(): Boolean = false
}
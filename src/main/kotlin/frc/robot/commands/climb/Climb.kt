package frc.robot.commands.climb

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.math.zeroIf
import frc.robot.RobotContainer
import frc.robot.commands.drive.Drive
import frc.robot.subsystems.*
import kotlin.math.absoluteValue


/**
 * Command that uses joysticks to control the climber extension.
 */
object Climb : CommandBase() {
    init {
        addRequirements(Climber)
        SmartDashboard.putBoolean("sync climb", true)
    }

    /**
     * Prepare the climber to climb.
     */
    override fun initialize() {
        Debug.log("Climb Command", "init", level = DebugFilter.Low)
    }
    /**
     * Set the winch percentage based on left/right joysticks
     */
    override fun execute() {
        Debug.log("Climb Command", "execute", level = DebugFilter.Low)

//        Climber.leftExtendable.percent = RobotContainer.controller.leftX.raw()
        val default = -RobotContainer.controller.rightY.raw().zeroIf { it.absoluteValue < .02 }
        Climber.leftWinch.percent = default
        if (SmartDashboard.getBoolean("sync climb", true)) {
            Climber.rightWinch.percent = default
        } else Climber.rightWinch.percent = -RobotContainer.controller.leftY.raw().zeroIf { it.absoluteValue < .02 }

        if(RobotContainer.controller.leftDPad.get()) Climber.highGrab.percent = 0.2
        else if(RobotContainer.controller.rightDPad.get()) Climber.highGrab.percent = -0.2
        else Climber.highGrab.stop()
    }

    override fun isFinished(): Boolean = false
}
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
        addRequirements(Climber, Turret)
        SmartDashboard.putBoolean("sync climb", true)
    }

    /**
     * Prepare the climber to climb.
     */
    override fun initialize() {
        Debug.log("Climb Command", "init", level = DebugFilter.Low)
        Turret.turret.position = 0.degrees
    }

    var hasFallen = false

    /**
     * Set the winch percentage based on left/right joysticks
     */
    override fun execute() {
        Debug.log("Climb Command", "execute", level = DebugFilter.Low)
        Turret.turret.position = 0.degrees
        if (Turret.turret.positionError.absoluteValue < 5.degrees) Climber.armsLifted = true

//        Climber.leftExtendable.percent = RobotContainer.controller.leftX.raw()
        val default = -RobotContainer.controller.rightY.raw().zeroIf { it.absoluteValue < .02 }
        Climber.leftWinch.percent = default
        Climber.rightWinch.percent = default
    }

    override fun isFinished(): Boolean = false
}
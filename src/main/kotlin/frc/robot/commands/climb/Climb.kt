package frc.robot.commands.climb

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.math.zeroIf
import frc.robot.Robot
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
        SmartDashboard.putBoolean("sync climb", true)
    }

    /**
     * Prepare the climber to climb.
     */
    override fun initialize() {
        Debug.log("Climb Command", "init", level = DebugFilter.Low)
        Turret.turret.position = 0.degrees
        Climber.status = ClimberStatus.ACTIVE
    }

    /**
     * true if the climb is too tilted and should have corrected earlier.
     * this should stop the climb becasue it could break the robot if we continue
     */
    var climbError = false;
    /**
     * is true if the climb has become too tilted and must be corrected.
     * it should stay that way until it is correct enough
     */
    var isCorrecting = false;
    var hasFallen = false

    /**
     * Set the winch percentage based on left/right joysticks
     */
    override fun execute() {
        Debug.log("Climb Command", "execute", level = DebugFilter.Low)
        Turret.turret.position = 0.degrees
        if(Turret.turret.positionError.absoluteValue < 5.degrees) Climber.staticsLifted = true

//        Climber.leftExtendable.percent = RobotContainer.controller.leftX.raw()
        val default = -RobotContainer.controller.rightY.raw().zeroIf { it.absoluteValue < .02 }

        // MARK: correcting code
        var tolorance = 3
        var degs = RobotContainer.gyro.roll.absoluteValue.degrees
        val defaultLeft = default * (degs*(1.0/tolorance))+1
        val defaultRight = default * (degs*(1.0/tolorance))-1

        Climber.rightWinch.percent = defaultRight
        if(SmartDashboard.getBoolean("sync climb", true)) {
            Climber.leftWinch.percent = defaultLeft
        } else {
            Climber.leftWinch.percent = -RobotContainer.controller.leftY.raw().zeroIf {
                it.absoluteValue < .02
            }
        }
//        Climber.rightExtendable.percent = RobotContainer.controller.rightX.raw()


        // set the status of the robot based on what the winches are doing
        if (Climber.leftWinch.percent.absoluteValue < 0.1) Climber.status = ClimberStatus.ACTIVE
        else if (Climber.leftWinch.percent < 0.0) {
            Climber.status = ClimberStatus.FALLING
            hasFallen = true
        } else if (Climber.leftWinch.percent.absoluteValue > 0.0) Climber.status = ClimberStatus.RISING

        if (RobotContainer.op.climbStabilization != 0.0 && Climber.leftWinch.linearPosition > 5.inches && hasFallen)
            Climber.stabalize()
        else Drive.execute()
    }

    override fun isFinished(): Boolean = false
}
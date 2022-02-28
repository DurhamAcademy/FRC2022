package frc.robot.commands.climb

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugLevel
import frc.robot.RobotContainer
import frc.robot.subsystems.*
import frc.kyberlib.math.filters.Differentiator
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.math.units.extensions.radiansPerSecond
import kotlin.math.absoluteValue


/**
 * Command that uses joysticks to control the climber extension.
 */
object Climb : CommandBase() {
    init {
        addRequirements(Climber, Drivetrain, Turret, Shooter)
    }

    private val swing = Differentiator()
    private const val dampeningConstant = 0.1  // random number outta my ass
    /**
     * Fun reaction wheel stuff. Optional, but cool if done
     */
    fun stabalize() {
        if (Climber.leftWinch.linearPosition < 5.inches) return
        val dTheta = swing.calculate(RobotContainer.gyro.pitch.radians).radiansPerSecond * -1.0

        Drivetrain.drive(DifferentialDriveWheelSpeeds( dTheta.value * dampeningConstant, dTheta.value * dampeningConstant))
        Shooter.flywheelMaster.torque = dTheta.value * dampeningConstant

    }

    /**
     * Prepare the climber to climb.
     */
    override fun initialize() {
        Debug.log("Climb Command", "init", level = DebugLevel.LowPriority)
        Turret.turret.position = 0.degrees
        Climber.staticsLifted = true
        Climber.status = ClimberStatus.ACTIVE

        Climber.leftExtendable.position = 90.degrees
        Climber.rightExtendable.position = 90.degrees
    }

    /**
     * Set the winch percentage based on left/right joysticks
     */
    override fun execute() {
        Debug.log("Climb Command", "execute", level = DebugLevel.LowPriority)
        Turret.turret.updateVoltage()

        Climber.leftWinch.percent = RobotContainer.controller.leftY.value / RobotContainer.controller.leftY.maxVal
        Climber.leftExtendable.percent = RobotContainer.controller.leftX.value / RobotContainer.controller.leftX.maxVal
        Climber.rightWinch.percent = RobotContainer.controller.rightY.value / RobotContainer.controller.rightY.maxVal
        Climber.rightExtendable.percent = RobotContainer.controller.rightX.value / RobotContainer.controller.rightX.maxVal


        // set the status of the robot based on what the winches are doing
        if (Climber.leftWinch.percent.absoluteValue < 0.1) Climber.status = ClimberStatus.ACTIVE
        else if (Climber.leftWinch.percent < 0.0) Climber.status = ClimberStatus.FALLING
        else if (Climber.leftWinch.percent.absoluteValue > 0.0) Climber.status = ClimberStatus.RISING
//        stabalize()
    }

    override fun isFinished(): Boolean = false
}
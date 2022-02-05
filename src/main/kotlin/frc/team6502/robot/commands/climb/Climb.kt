package frc.team6502.robot.commands.climb

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.RobotContainer
import frc.team6502.robot.subsystems.*
import kyberlib.math.filters.Differentiator
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.radiansPerSecond
import kotlin.math.absoluteValue


object Climb : CommandBase() {
    init {
        addRequirements(Climber, Drivetrain, Turret, Shooter)
    }

    val swing = Differentiator()
    const val dampeningConstant = 0.1  // random number outta my ass
    /**
     * Fun reaction wheel stuff. Optional, but cool if done
     */
    fun stabalize() {
        val dTheta = swing.calculate(RobotContainer.gyro.pitch.radians).radiansPerSecond * -1.0

        Drivetrain.drive(DifferentialDriveWheelSpeeds( dTheta.value * dampeningConstant, dTheta.value * dampeningConstant))
        Shooter.flywheelMaster.voltage = dTheta.value * dampeningConstant
    }

    override fun initialize() {
        // change led colors to reflect new mode
        Turret.fieldRelativeAngle = 0.degrees
        Climber.armsLifted = true
        Climber.status = CLIMBER_STATUS.ACTIVE
    }

    override fun execute() {
        Climber.leftWinch.percent = RobotContainer.controller.leftY.value / RobotContainer.controller.leftY.maxVal
        Climber.rightWinch.percent = RobotContainer.controller.rightY.value / RobotContainer.controller.rightY.maxVal
        if (Climber.leftWinch.percent.absoluteValue < 0.1) Climber.status = CLIMBER_STATUS.ACTIVE
        else if (Climber.leftWinch.percent < 0.0) Climber.status = CLIMBER_STATUS.FALLING
        else if (Climber.leftWinch.percent.absoluteValue > 0.0) Climber.status = CLIMBER_STATUS.RISING
        stabalize()
    }
}
package frc.team6502.robot.commands.climb

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.RobotContainer
import frc.team6502.robot.subsystems.Climber
import frc.team6502.robot.subsystems.Drivetrain
import frc.team6502.robot.subsystems.Shooter
import frc.team6502.robot.subsystems.Turret
import kyberlib.math.units.extensions.degrees


object Climb : CommandBase() {
    init {
        addRequirements(Climber, Drivetrain, Turret, Shooter)
    }

    /**
     * Fun reaction wheel stuff. Optional, but cool if done
     */
    fun stabalize() {}

    override fun initialize() {
        // change led colors to reflect new mode
        Turret.fieldRelativeAngle = 0.degrees
    }

    override fun execute() {
        Climber.leftWinch.percent = RobotContainer.controller.leftY.value / RobotContainer.controller.leftY.maxVal
        Climber.rightWinch.percent = RobotContainer.controller.rightY.value / RobotContainer.controller.rightY.maxVal
        stabalize()
    }
}
package frc.robot.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.subsystems.Drivetrain

object Operator: CommandBase() {
    init {
        addRequirements(Drivetrain)
    }
    override fun execute() {
        val linearmotion = RobotContainer.controller.leftY.value
        val angularmotion = RobotContainer.controller.leftX.value

        Drivetrain.drive(ChassisSpeeds(linearmotion, 0.0, angularmotion))
    }
}
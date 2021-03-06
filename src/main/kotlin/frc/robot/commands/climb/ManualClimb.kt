package frc.robot.commands.climb

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.inches
import frc.robot.subsystems.Climber

object ManualClimb : CommandBase() {
    const val leftWinchString = "left winch volt"
    const val rightWinchString = "right winch volt"

    //    const val leftRotString = "left winch volt"
//    const val rightRotString = "right winch volt"
    const val staticString = "statics"

    init {
        addRequirements(Climber)
        SmartDashboard.putNumber(leftWinchString, 0.0)
        SmartDashboard.putNumber(rightWinchString, 0.0)
        SmartDashboard.putBoolean(staticString, false)
    }

    override fun execute() {
        Climber.leftWinch.voltage = SmartDashboard.getNumber(leftWinchString, 0.0)
        Climber.rightWinch.voltage = SmartDashboard.getNumber(rightWinchString, 0.0)
        Climber.staticsLifted = SmartDashboard.getBoolean(staticString, false)

        SmartDashboard.putNumber("right arm pos", Climber.rightWinch.linearPosition.inches)
    }

    override fun end(interrupted: Boolean) {
        Climber.leftWinch.voltage = 0.0
        Climber.rightWinch.voltage = 0.0
    }
}
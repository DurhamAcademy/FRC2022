package frc.robot.commands.climb

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Climber

object ManualClimb : CommandBase() {
    private const val leftWinchString = "left winch volt"
    private const val rightWinchString = "right winch volt"

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
        Climber.setClimbPercents(
            SmartDashboard.getNumber(leftWinchString, 0.0) / 12.0,
            SmartDashboard.getNumber(rightWinchString, 0.0) / 12.0
        )
        Climber.armsLifted = SmartDashboard.getBoolean(staticString, false)
    }

    override fun end(interrupted: Boolean) {
        Climber.setClimbPercents(0.0, 0.0)
    }
}
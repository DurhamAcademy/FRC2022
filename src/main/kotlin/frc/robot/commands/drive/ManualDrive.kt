package frc.robot.commands.drive

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain

object ManualDrive : CommandBase() {
    private const val leftString = "leftPercent"
    private const val rightString = "rightPercent"

    init {
        addRequirements(Drivetrain)
        SmartDashboard.putNumber(leftString, 0.0)
        SmartDashboard.putNumber(rightString, 0.0)
    }

    override fun execute() {
        if(Drivetrain.driveInversion) {
            Drivetrain.leftFollower.percent = SmartDashboard.getNumber(leftString, 0.0)
            Drivetrain.rightFollower.percent = SmartDashboard.getNumber(rightString, 0.0)
        } else {
            Drivetrain.leftMaster.percent = SmartDashboard.getNumber(leftString, 0.0)
            Drivetrain.rightMaster.percent = SmartDashboard.getNumber(rightString, 0.0)
        }
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}
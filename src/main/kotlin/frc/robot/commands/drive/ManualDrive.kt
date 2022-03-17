package frc.robot.commands.drive

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain

object ManualDrive : CommandBase() {
    init {
        addRequirements(Drivetrain)
    }

    const val fwdString = "fwd"
    const val turnString = "turn"

    override fun initialize() {
        SmartDashboard.putNumber(fwdString, 0.0)
        SmartDashboard.putNumber(turnString, 0.0)
    }

    override fun execute() {
        Drivetrain.drive(ChassisSpeeds(
            SmartDashboard.getNumber(fwdString, 0.0),
            0.0,
            SmartDashboard.getNumber(turnString, 0.0)
        ))
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}
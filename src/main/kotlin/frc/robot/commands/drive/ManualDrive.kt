package frc.robot.commands.drive

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain

object ManualDrive : CommandBase() {
    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        SmartDashboard.putNumber("fwd", 0.0)
        SmartDashboard.putNumber("turn", 0.0)
    }

    override fun execute() {
        Drivetrain.drive(ChassisSpeeds(
            SmartDashboard.getNumber("fwd", 0.0),
            0.0,
            SmartDashboard.getNumber("turn", 0.0)
        ))
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}
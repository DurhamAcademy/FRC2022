package frc.kyberlib.auto

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.auto.trajectory.KTrajectory
import frc.kyberlib.mechanisms.drivetrain.dynamics.KDriveDynamics

class AutoDrive(private val path: KTrajectory) : CommandBase() {
    override fun execute() {
        KDriveDynamics.instance!!.drive(path)
    }
}
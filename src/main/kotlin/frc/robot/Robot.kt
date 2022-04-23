package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import frc.kyberlib.command.KRobot
import frc.robot.subsystems.Drivetrain

class Robot : KRobot() {
    override fun robotInit() {
        RobotContainer  // initializes object
    }


    override fun disabledInit() {
        Drivetrain.stop()
    }

    private fun reset(pose: Pose2d) {
        // reset navigation to a specific pose
        RobotContainer.navigation.pose = pose
    }
}
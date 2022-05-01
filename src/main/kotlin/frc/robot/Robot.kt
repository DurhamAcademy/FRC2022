package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.kyberlib.command.KRobot
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.w
import frc.kyberlib.math.units.string
import frc.kyberlib.simulation.field.KField2d
import frc.robot.subsystems.Drivetrain

class Robot : KRobot() {
    override fun robotInit() {
        println("init")
        RobotContainer  // initializes object
    }

    override fun disabledInit() {
        Drivetrain.stop()
    }

    override fun robotPeriodic() {
        KField2d.robotPose = RobotContainer.navigation.pose
        SmartDashboard.putString("pose", RobotContainer.navigation.pose.string)
    }

    override fun autonomousInit() {
        reset(Pose2d(0.0, 0.0, 0.degrees.w))
    }

    private fun reset(pose: Pose2d) {
        // reset navigation to a specific pose
        RobotContainer.navigation.pose = pose
    }
}
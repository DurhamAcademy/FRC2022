package frc.robot

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.kyberlib.auto.Navigator
import frc.kyberlib.auto.trajectory.TrajectoryManager
import frc.kyberlib.input.controller.KXboxController
import frc.kyberlib.sensors.gyros.KNavX
import frc.robot.subsystems.Drivetrain

/**
 * Contains all Robot subsystems and sensors
 */
object RobotContainer {
    // initialize sensors and inputs here
    val gyro = KNavX()
    val navigation = Navigator(gyro, Constants.START_POSE)

    // controllers
    val controller = KXboxController(0).apply {}

    init {
        // initialize subsystems here:
        Drivetrain
    }

}
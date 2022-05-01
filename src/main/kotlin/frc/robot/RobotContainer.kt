package frc.robot

import frc.kyberlib.auto.Navigator
import frc.kyberlib.input.controller.KXboxController
import frc.kyberlib.sensors.gyros.KPigeon
import frc.robot.subsystems.Drivetrain
import kotlin.math.PI

/**
 * Contains all Robot subsystems and sensors
 */
object RobotContainer {
    // initialize sensors and inputs here
    val gyro = KPigeon(0)
    val navigation = Navigator(gyro, Constants.START_POSE)

    // controllers
    val controller = KXboxController(0).apply {
        rightX.apply {
            maxVal = -3.0
        }
        rightY.apply {
            maxVal = -3.0
        }
        leftX.apply {
            maxVal = 3 * PI
        }
    }

    init {
        // initialize subsystems here:
        Drivetrain
    }

}
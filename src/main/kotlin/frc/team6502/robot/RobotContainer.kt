package frc.team6502.robot

import edu.wpi.first.wpilibj.DigitalInput
import frc.team6502.robot.subsystems.Climber
import frc.team6502.robot.subsystems.Drivetrain
import frc.team6502.robot.subsystems.Intaker
import frc.team6502.robot.subsystems.Shooter
import kyberlib.auto.Navigator
import kyberlib.input.controller.KXboxController
import kyberlib.sensors.gyros.KPigeon
import org.photonvision.PhotonCamera
import kotlin.math.PI

/**
 * Contains all Robot subsystems and sensors
 */
object RobotContainer {
   // initialize sensors and inputs here
    val gyro = KPigeon(6)
    val limelight = PhotonCamera("gloworm")
    val turretLimit = DigitalInput(0)

    val controller = KXboxController(0).apply {
        // steering
        rightX.apply {
            maxVal = -5 * PI
            expo = 73.0
            deadband = 0.1
        }

        // throttle
        leftY.apply {
            maxVal = -12.0
            expo = 20.0
            deadband = 0.2
        }
    }


    val navigation = Navigator(gyro)

    init {
        // initialize subsystems here:
        Drivetrain
        Shooter
        Climber
        Intaker
    }

}
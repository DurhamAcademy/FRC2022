package frc.robot

import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.cscore.VideoMode
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.kyberlib.auto.Navigator
import frc.kyberlib.auto.trajectory.TrajectoryManager
import frc.kyberlib.command.Game
import frc.kyberlib.input.controller.KXboxController
import frc.kyberlib.math.units.extensions.seconds
import frc.kyberlib.sensors.gyros.KPigeon
import frc.robot.commands.climb.Climb
import frc.robot.commands.intake.Intake
import frc.robot.commands.shooter.Shoot
import frc.robot.commands.turret.ZeroTurret
import frc.robot.subsystems.*
import org.photonvision.SimPhotonCamera

/**
 * Contains all Robot subsystems and sensors
 */
object RobotContainer {
    // initialize sensors and inputs here
    val gyro = KPigeon(6)
    val limelight = SimPhotonCamera("gloworm")

    init {
        if (Game.real) {
            val video = CameraServer.startAutomaticCapture()
            video.videoMode = VideoMode(video.videoMode.pixelFormat, 640, 480, 30)
        }
    }

    var startTime = 0.seconds

    // hall sensor
    val turretLimit = DigitalInput(0)

    val navigation = Navigator(gyro, Constants.START_POSE)

    // controllers
    val controller = KXboxController(0).apply {
        aButton.debounce(.3, Debouncer.DebounceType.kFalling).whileActiveOnce(Intake)
        bButton.whenHeld(Shoot)
        downDPad.whenPressed(ZeroTurret)
        upDPad.toggleWhenPressed(Climb)
    }

    // auto path chooser
    val autoChooser = SendableChooser<String>().apply {
        val options = TrajectoryManager.routines
        for (path in options) addOption(path, path)
        setDefaultOption("Default", "Default")
        SmartDashboard.putData("auto", this)
    }

    init {
        // initialize subsystems here:
        Climber
        Conveyor
        Drivetrain
        Intaker
        Shooter
        Turret
        Limelight
    }

}
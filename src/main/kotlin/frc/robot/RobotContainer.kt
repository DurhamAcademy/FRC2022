package frc.robot

import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.cscore.VideoMode
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.kyberlib.auto.Navigator
import frc.kyberlib.auto.trajectory.TrajectoryManager
import frc.kyberlib.command.Game
import frc.kyberlib.input.controller.KXboxController
import frc.kyberlib.lighting.KLEDRegion
import frc.kyberlib.lighting.KLEDStrip
import frc.kyberlib.lighting.animations.*
import frc.kyberlib.math.kEpsilon
import frc.kyberlib.math.units.extensions.feetPerSecond
import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.math.units.extensions.seconds
import frc.kyberlib.sensors.gyros.KPigeon
import frc.robot.commands.climb.Climb
import frc.robot.commands.climb.PrepareClimb
import frc.robot.commands.conveyor.Eject
import frc.robot.commands.intake.Flush
import frc.robot.commands.intake.Intake
import frc.robot.commands.shooter.ForceShoot
import frc.robot.commands.shooter.Shoot
import frc.robot.controls.DefaultControls
import frc.robot.controls.OperatorControls
import frc.robot.subsystems.*

/**
 * Contains all Robot subsystems and sensors
 */
object RobotContainer {
    // initialize sensors and inputs here
    val gyro = KPigeon(6)
    val limelight = frc.kyberlib.sensors.Limelight()

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
    val controller = KXboxController(0)  // xbox
    val op = OperatorControls()  // dashboard controls

    private val controlType = DefaultControls
    var controlScheme = controlType.apply {
        INTAKE.debounce(.3, Debouncer.DebounceType.kFalling).whileActiveOnce(Intake)
        SHOOT.whenHeld(Shoot())
        FORCE_SHOT.whenHeld(ForceShoot)
        EJECT.whenHeld(Eject)
        FLUSH.whenHeld(Flush)
        CLIMB_MODE.toggleWhenPressed(Climb)
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
        Limelight
    }

}
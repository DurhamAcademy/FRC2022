package frc.robot

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.kyberlib.auto.Navigator
import frc.kyberlib.auto.TrackingMode
import frc.kyberlib.auto.trajectory.TrajectoryManager
import frc.kyberlib.command.Game
import frc.kyberlib.input.controller.KXboxController
import frc.kyberlib.lighting.KLEDRegion
import frc.kyberlib.lighting.KLEDStrip
import frc.kyberlib.lighting.animations.AnimationBlink
import frc.kyberlib.lighting.animations.AnimationCylon
import frc.kyberlib.lighting.animations.AnimationPulse
import frc.kyberlib.lighting.animations.AnimationSolid
import frc.kyberlib.sensors.gyros.KPigeon
import frc.robot.commands.Emote
import frc.robot.commands.climb.Climb
import frc.robot.commands.conveyor.Eject
import frc.robot.commands.intake.Flush
import frc.robot.commands.intake.Intake
import frc.robot.commands.shooter.Dispose
import frc.robot.commands.shooter.ForceShoot
import frc.robot.commands.shooter.Shoot
import frc.robot.commands.turret.AimTurret
import frc.robot.commands.turret.FreezeTurret
import frc.robot.commands.turret.SeekTurret
import frc.robot.commands.turret.ZeroTurret
import frc.robot.controls.DefaultControls
import frc.robot.controls.OperatorControls
import frc.robot.controls.RocketLeague
import frc.robot.subsystems.*
import org.photonvision.PhotonCamera
import java.awt.Color

/**
 * Contains all Robot subsystems and sensors
 */
object RobotContainer {
    // initialize sensors and inputs here
    val gyro = KPigeon(6)
    val limelight = PhotonCamera("gloworm")
    val ballMonitor = PhotonCamera("balls")
    init {
//        if(Game.real && false) { //Constants.DRIVER_CAMERA) {
//            val video = CameraServer.startAutomaticCapture()
//            video.videoMode = VideoMode(video.videoMode.pixelFormat, 640, 480, 30)
//        }
    }
    // hall sensor
    val turretLimit = DigitalInput(0)

    val navigation = Navigator(gyro, Constants.START_POSE, trackingMode = TrackingMode.Fancy)

    // controllers
    val controller = KXboxController(0)  // xbox
    val op = OperatorControls()  // dashboard controls

    var controlScheme = RocketLeague.apply {
        INTAKE.debounce(.3, Debouncer.DebounceType.kFalling).whileActiveOnce(Intake)
        SHOOT.whileActiveOnce(Shoot)
        FORCE_SHOT.whileActiveOnce(ForceShoot)
        EJECT.whileActiveOnce(Eject)
        FLUSH.whileActiveOnce(Flush)
        LOCK_TURRET.toggleWhenActive(FreezeTurret)
        ZERO_TURRET.whenActive(ZeroTurret)
        CLIMB_MODE.toggleWhenActive(Climb)
        EMOTE.whileActiveOnce(Emote)
        DISPOSE.whileActiveOnce(Dispose)
    }

    // auto path chooser
    val autoChooser = SendableChooser<String>().apply {
        val options = TrajectoryManager.routines
        for (path in options) addOption(path, path)
        setDefaultOption("Default", "Default")
        SmartDashboard.putData("auto", this)
    }

    // QUOTE: I dont need a christmas tree, i need a robot. -Cherith
    val leds = KLEDStrip(0, 14).apply {
        val coral = Color(255, 93, 115)

        // idle alliance animations
        this += KLEDRegion(AnimationCylon(Color.RED, 5, 40), 0, 14) { Game.alliance == DriverStation.Alliance.Red }
        this += KLEDRegion(AnimationCylon(Color.CYAN, 5, 40), 0, 14) { Game.alliance == DriverStation.Alliance.Blue }

        // turret status
        this += KLEDRegion(AnimationBlink(Color.BLUE, 20), 0, 14) { Turret.currentCommand == ZeroTurret }
        this += KLEDRegion(AnimationSolid(Color.RED), 0, 14) { Turret.currentCommand == SeekTurret }
        this += KLEDRegion(AnimationSolid(Color.YELLOW), 0, 14) { Turret.currentCommand == AimTurret }
        this += KLEDRegion(AnimationPulse(Color.YELLOW, 40), 0, 14, false) { Turret.ready }
        // shooter status
        this += KLEDRegion(AnimationSolid(Color.GREEN), 0, 14, false) { Shooter.status == ShooterStatus.SPINUP }
        this += KLEDRegion(
            AnimationPulse(Color.GREEN, 40),
            0,
            14,
            false
        ) { Shooter.status == ShooterStatus.SHOT }
        this += KLEDRegion(
            AnimationPulse(Color.RED, 40),
            0,
            14
        ) { !Shooter.inRange && Turret.currentCommand == AimTurret }
    }

    init {
        // initialize subsystems here:
        Climber
        Conveyor
        Drivetrain
        Intaker
        Shooter
        Turret
    }

}
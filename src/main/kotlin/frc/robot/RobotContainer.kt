package frc.robot

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.kyberlib.auto.Navigator
import frc.kyberlib.auto.TrackingMode
import frc.kyberlib.auto.trajectory.TrajectoryManager
import frc.kyberlib.input.controller.KXboxController
import frc.kyberlib.lighting.KLEDRegion
import frc.kyberlib.lighting.KLEDStrip
import frc.kyberlib.lighting.animations.AnimationCylon
import frc.kyberlib.sensors.gyros.KPigeon
import frc.robot.commands.Emote
import frc.robot.commands.intake.Eject
import frc.robot.commands.intake.Flush
import frc.robot.commands.intake.Intake
import frc.robot.commands.shooter.ShooterCalibration
import frc.robot.commands.shooter.Shoot
import frc.robot.commands.turret.FreezeTurret
import frc.robot.subsystems.Drivetrain
import frc.robot.controls.ControlSchema2022
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
    val turretLimit = DigitalInput(0)

    val navigation = Navigator(gyro, Constants.START_POSE, trackingMode = if(Constants.NAVIGATION_CORRECTION) TrackingMode.Fancy else if (Constants.DUMB_NAVIGATION) TrackingMode.DumbBoth else TrackingMode.Fast)

    val controller = KXboxController(0)
    val op = OperatorControls()

    private val schemaChooser = SendableChooser<ControlSchema2022>().apply {
        setDefaultOption("Default", DefaultControls)
        addOption("RocketLeague", RocketLeague)
        SmartDashboard.putData("control system", this)
    }
    val controlScheme = DefaultControls.apply {
        INTAKE.debounce(.3, Debouncer.DebounceType.kFalling).whileActiveOnce(Intake)
        SHOOT.whileActiveOnce(Shoot)
        FORCE_SHOT.whileActiveOnce(ShooterCalibration)
        EJECT.whileActiveOnce(Eject)
        FLUSH.whileActiveOnce(Flush)
        LOCK_TURRET.toggleWhenActive(FreezeTurret)
//        CLIMB_MODE.toggleWhenActive(Climb)
        EMOTE.whileActiveOnce(Emote)
    }

    val autoChooser = SendableChooser<String>().apply {
        val options = TrajectoryManager.routines
        for (path in options) addOption(path, path)
        setDefaultOption("Default", "Default")
        SmartDashboard.putData("auto", this)
    }
// QUOTE: I dont need a christmas tree, i need a robot. -Cherith
    val leds = KLEDStrip(0, 14).apply {
        val coral = Color(255, 93, 115)
        this += KLEDRegion(AnimationCylon(coral, 3, 20),0, 14)
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
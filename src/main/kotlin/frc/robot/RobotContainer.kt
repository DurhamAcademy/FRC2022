package frc.robot

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.kyberlib.auto.Navigator
import frc.kyberlib.auto.TrackingMode
import frc.kyberlib.auto.trajectory.TrajectoryManager
import frc.kyberlib.input.controller.KXboxController
import frc.kyberlib.lighting.KLEDRegion
import frc.kyberlib.lighting.KLEDStrip
import frc.kyberlib.lighting.animations.AnimationPulse
import frc.kyberlib.lighting.animations.AnimationSilon
import frc.kyberlib.pneumatics.KSolenoid
import frc.kyberlib.sensors.gyros.KPigeon
import frc.robot.commands.Emote
import frc.robot.commands.intake.Flush
import frc.robot.commands.intake.Intake
import frc.robot.commands.shooter.ForceShoot
import frc.robot.commands.shooter.ShooterCalibration
import frc.robot.commands.shooter.Shoot
import frc.robot.commands.turret.FreezeTurret
import frc.robot.subsystems.Drivetrain
import frc.robot.controls.ControlSchema2022
import frc.robot.controls.DefaultControls
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

    private val schemaChooser = SendableChooser<ControlSchema2022>().apply {
        setDefaultOption("Default", DefaultControls)
        addOption("RocketLeague", RocketLeague)
        SmartDashboard.putData("control system", this)
    }
    val controlScheme = DefaultControls.apply {
        INTAKE.debounce(.3, Debouncer.DebounceType.kFalling).whileActiveOnce(Intake)
        SHOOT.whileActiveOnce(Shoot)
        FORCE_SHOT.whileActiveOnce(ForceShoot)
//        EJECT.whileActiveOnce(Eject)
        FLUSH.whileActiveOnce(Flush)
        LOCK_TURRET.toggleWhenActive(FreezeTurret)
//        CLIMB_MODE.toggleWhenActive(Climb)
        EMOTE.whileActiveOnce(Emote)
        FLYWHEEL_INCREASE.whenActive {
            Shooter.shooterMult += .01
            emptySet<Subsystem>()
        }
        FLYWHEEL_DECREASE.whenActive {
            Shooter.shooterMult -= .01
            emptySet<Subsystem>()
        }
        COMPRESSOR_TOGGLE.whileActiveOnce {
            if(KSolenoid.compressor.enabled()) KSolenoid.compressor.disable() else KSolenoid.compressor.enableDigital()
            emptySet<Subsystem>()
        }
    }

    val autoChooser = SendableChooser<String>().apply {
        val options = TrajectoryManager.routines
        for (path in options) addOption(path, path)
        setDefaultOption("Default", "Default")
        SmartDashboard.putData("auto", this)
    }
// QUOTE: I dont need a christmas tree, i need a robot. -Cherith
    val leds = KLEDStrip(0, 103).apply {
        val coral = Color(255, 93, 115)
        this += KLEDRegion(AnimationSilon(coral, 100, 100),0, 100){true}
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
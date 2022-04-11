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
import frc.robot.commands.Emote
import frc.robot.commands.climb.Climb
import frc.robot.commands.climb.PrepareClimb
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
import kotlin.math.absoluteValue

/**
 * Contains all Robot subsystems and sensors
 */
object RobotContainer {
    // initialize sensors and inputs here
    val gyro = KPigeon(6)
    val limelight = PhotonCamera("gloworm")
    val ballMonitor = PhotonCamera("balls")

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

    private val controlType = if (Game.sim) DefaultControls else RocketLeague
    var controlScheme = controlType.apply {
        INTAKE.debounce(.3, Debouncer.DebounceType.kFalling).whileActiveOnce(Intake)
        SHOOT.whenHeld(Shoot)
        FORCE_SHOT.whenHeld(ForceShoot)
        EJECT.whenHeld(Eject)
        FLUSH.whenHeld(Flush)
        LOCK_TURRET.toggleWhenPressed(FreezeTurret)
        ZERO_TURRET.whenPressed(ZeroTurret)
        CLIMB_MODE.toggleWhenPressed(Climb)
        EMOTE.whenHeld(Emote)
        DISPOSE.whenHeld(Dispose)
    }

    // auto path chooser
    val autoChooser = SendableChooser<String>().apply {
        val options = TrajectoryManager.routines
        for (path in options) addOption(path, path)
        setDefaultOption("Default", "Default")
        SmartDashboard.putData("auto", this)
    }

    // QUOTE: I dont need a christmas tree, i need a robot. -Cherith
    val leds = KLEDStrip(1, 74).apply {
        val coral = Color(255, 93, 115)
        val allianceColor = if (Game.alliance == DriverStation.Alliance.Red) coral else Color.CYAN

        // idle alliance animations
        val prematchArms = AnimationRGBFade(7.seconds)//AnimationRGBWave(1.0, .1.seconds)
        val idleArms = AnimationSolid(Color.BLACK) { Game.enabled }
        val idleCylon = AnimationCylon(allianceColor, 5, 1.seconds)

        // turret status
        val zero = AnimationBlink(Color.BLUE, 1.seconds) { Turret.currentCommand == ZeroTurret }
        val seek = AnimationSolid(Color.RED) { Turret.currentCommand == SeekTurret && Game.enabled }
        val aiming = AnimationSolid(Color.YELLOW) { Turret.currentCommand == AimTurret }
        val turretReady = AnimationPulse(Color.YELLOW, 2.seconds) { Turret.ready && Game.enabled }

        // shooter status
        val spinup = AnimationSolid(Color.GREEN) { Shooter.status == ShooterStatus.SPINUP }
        val shooting = AnimationPulse(Color.GREEN, 2.seconds) { Shooter.status == ShooterStatus.SHOT }
        val outOfRange = AnimationPulse(Color.RED, 2.seconds) { !Shooter.inRange && Turret.currentCommand == AimTurret }

        // climb
        val climbColor = Color(255, 155, 0)
        val upClimb = AnimationRain(climbColor, 10, .1.seconds, false) { Climber.leftWinch.percent < -kEpsilon }
        val downClimb = AnimationRain(climbColor, 10, .1.seconds, true) { Climber.leftWinch.percent > kEpsilon }
        val extension =
            AnimationCustom(
                { t, l -> List<Color>(l) { index -> if (index / l.toDouble() < Climber.extension / 24.inches) climbColor else Color.BLACK } },
                { Climber.staticsLifted },
                false
            )
        val prepare = AnimationRain(climbColor, 3, 1.seconds) { Climber.currentCommand == PrepareClimb }
        val postMatch =
            AnimationPulse(allianceColor, 2.seconds) { Game.disabled && Game.COMPETITION && startTime != 0.seconds }

        // other random animations
        val leftTurn = AnimationBlink(Color.YELLOW, .5.seconds) { Drivetrain.chassisSpeeds.omegaRadiansPerSecond > 0.1 }
        val rightTurn =
            AnimationBlink(Color.YELLOW, .5.seconds) { Drivetrain.chassisSpeeds.omegaRadiansPerSecond < -0.1 }

        val maxSpeed = 12.feetPerSecond
        val leftSpeed = AnimationCustom({ t, l ->
            List<Color>(l) { index ->
                val percentDis = (index / l.toDouble())
                val percentSpeed = (Drivetrain.leftMaster.linearVelocity / maxSpeed)
                if (percentDis + .01 < percentSpeed.absoluteValue) {
                    Color(Color.HSBtoRGB((.33 - .33 * percentDis).toFloat(), 1F, 1F))
                } else Color.BLACK
            }
        }, { Game.enabled })
        val rightSpeed = AnimationCustom({ t, l ->
            List<Color>(l) { index ->
                val percentDis = (index / l.toDouble())
                val percentSpeed = (Drivetrain.rightMaster.linearVelocity / maxSpeed)
                if (percentDis + .01 < percentSpeed.absoluteValue) {
                    Color(Color.HSBtoRGB((.33 - .33 * percentDis).toFloat(), 1F, 1F))
                } else Color.BLACK
            }
        }, { Game.enabled })

        // left arm
        this += KLEDRegion(
            0, 30,
            prematchArms, idleArms, leftSpeed, upClimb, downClimb, prepare, postMatch
        )
        // right arm
        this += KLEDRegion(
            30, 60,
            prematchArms, idleArms, rightSpeed, upClimb, downClimb, prepare, postMatch
        )
//         turret
        this += KLEDRegion(
            60, 74,
            idleCylon, zero, seek, aiming, turretReady, spinup, shooting, outOfRange
        )
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
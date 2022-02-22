package frc.robot

import edu.wpi.first.wpilibj.DigitalInput
import frc.kyberlib.auto.Navigator
import frc.kyberlib.input.controller.KXboxController
import frc.kyberlib.sensors.gyros.KPigeon
import frc.robot.subsystems.*
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

    val navigation = Navigator(gyro, Constants.START_POSE)

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

//        leftTrigger.activateAt(0.5).whileActiveOnce(Intake)
//        leftBumper.whileActiveOnce(Flush)
//
//        yButton.toggleWhenActive(fullAutoClimb)
//        aButton.toggleWhenActive(Climb)
//        bButton.whileActiveOnce(ToggleArmLift())
    }

//    val leds = KLEDStrip(0, 103).apply {
//        val length = 103
//        val coral = Color(255, 93, 115)
//
//        this += KLEDRegion(AnimationRGBWave(0.5, 2, false), 0, length)  // default
//
//        // climb
//        val climbColor = Color(255, 255, 0)
//        val climbStart = 0
//        val climbLength = 100
//        this += KLEDRegion(AnimationSolid(climbColor), climbStart, climbLength)
//            {Climber.status == CLIMBER_STATUS.IDLE}
//        this += KLEDRegion(AnimationBlink(climbColor, 5), climbStart, climbLength)
//            {Climber.status == CLIMBER_STATUS.ACTIVE}
//        this += KLEDRegion(AnimationRain(climbColor, 5, 2, false), climbStart, climbLength)
//            {Climber.status == CLIMBER_STATUS.FALLING}
//        this += KLEDRegion(AnimationRain(climbColor, 5, 2, true), climbStart, climbLength)
//            {Climber.status == CLIMBER_STATUS.RISING}
//
//        // conveyor
//        val allianceColor = when(Game.alliance){
//            DriverStation.Alliance.Blue -> Color.BLUE
//            DriverStation.Alliance.Red -> Color.RED
//            else -> Color.BLUE
//        }
//        val enemyColor = when(Game.alliance){
//            DriverStation.Alliance.Blue -> Color.RED
//            DriverStation.Alliance.Red -> Color.BLUE
//            else -> Color.RED
//        }
//        val conveyorStart = 0
//        val conveyorLength = 50
//        this += KLEDRegion(AnimationSolid(coral), conveyorStart, conveyorLength)
//            {Conveyor.status == CONVEYOR_STATUS.EMPTY}
//        this += KLEDRegion(AnimationBlink(allianceColor, 5), conveyorStart, conveyorLength)
//            {Conveyor.status == CONVEYOR_STATUS.SINGLE_GOOD}
//        this += KLEDRegion(AnimationSolid(allianceColor), conveyorStart, conveyorLength)
//            {Conveyor.status == CONVEYOR_STATUS.FULL_GOOD}
//        this += KLEDRegion(AnimationSolid(enemyColor), conveyorStart, conveyorLength)
//            {Conveyor.status == CONVEYOR_STATUS.BAD}
//        this += KLEDRegion(AnimationPulse(allianceColor, 2), conveyorStart, conveyorLength)
//            {Conveyor.feeder.percent > 0.01}
//
//        // intake
//        val intakeStart = 0
//        val intakeLength = 100
//        this += KLEDRegion(AnimationSolid(coral), intakeStart, intakeLength) {Intaker.deployed}
//
//        // Shooter & Turret
//        val shooterStart = 0
//        val shooterLength = 100
//        this += KLEDRegion(AnimationBlink(Color(255, 0, 0), 5), shooterStart, shooterLength)
//            {Turret.status == TURRET_STATUS.LOST}
//        this += KLEDRegion(AnimationBlink(Color(255, 120, 0), 5), shooterStart, shooterLength)
//            {Turret.status == TURRET_STATUS.NOT_FOUND}
//        this += KLEDRegion(AnimationSolid(Color(255, 255, 0)), shooterStart, shooterLength)
//            {Turret.status == TURRET_STATUS.ADJUSTING}
//        this += KLEDRegion(AnimationBlink(Color(0, 255, 0), 5), shooterStart, shooterLength)
//            {Turret.status == TURRET_STATUS.LOCKED && Shooter.status != SHOOTER_STATUS.HIGH_READY}
//        this += KLEDRegion(AnimationSolid(Color(0, 255, 0)), shooterStart, shooterLength)
//            {Turret.status == TURRET_STATUS.LOCKED && Shooter.status == SHOOTER_STATUS.HIGH_READY}
//        this += KLEDRegion(AnimationPulse(Color(0, 255, 0), 5), shooterStart, shooterLength)
//            {Turret.status == TURRET_STATUS.LOCKED && Shooter.status == SHOOTER_STATUS.AUTO_SHOT}
//        this += KLEDRegion(AnimationPulse(Color(255, 120, 0), 5), shooterStart, shooterLength)
//            {Shooter.status == SHOOTER_STATUS.FORCE_SHOT}
//
//    }

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
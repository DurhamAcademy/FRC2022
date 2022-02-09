package frc.robot.subsystems

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.shooter.Shoot
import frc.robot.subsystems.Turret.targetLost
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.mechanisms.Flywheel
import frc.kyberlib.motorcontrol.rev.KSparkMax


/**
 * Current status of the shooter mechanism
 */
enum class SHOOTER_STATUS {
    IDLE, SPINUP, LOW_READY, HIGH_READY, AUTO_SHOT, FORCE_SHOT
}


/**
 * Encapsulates all the things relevant to shooting the ball
 */
object Shooter : SubsystemBase(), Debug {
    var status = SHOOTER_STATUS.IDLE

    // main motor attached to the flywheel
    val flywheelMaster = KSparkMax(0).apply {
        identifier = "flywheel"
        radius = Constants.FLYWHEEL_RADIUS
        Notifier{this.velocity = this.velocitySetpoint}.startPeriodic(.002)  // todo: test this
    }
    val flywheelControl = Flywheel(flywheelMaster, Constants.FLYWHEEL_MOMENT_OF_INERTIA, 4)
    // additional motors that copy the main
    private val flywheel2 = KSparkMax(0).apply { follow(flywheelMaster) }
    private val flywheel3 = KSparkMax(0).apply { follow(flywheelMaster) }
    private val flywheel4 = KSparkMax(0).apply { follow(flywheelMaster) }

    // Servo that sets the hood angle
    private val hood = Servo(1)

    // todo: figure out this will work - it won't figure it out
    var hoodAngle: Angle
        get() = hood.get().degrees
        set(value) {
            hood.set(value.degrees)
        }

    // how far from the hub the robot is (based on limelight)
    private val distanceFilter = LinearFilter.movingAverage(8)
    val targetDistance: Length? = if (Game.sim) RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION).meters
                                else if (targetLost) null
                                else distanceFilter.calculate(((Constants.UPPER_HUB_HEIGHT - Constants.LIMELIGHT_HEIGHT) / (Constants.LIMELIGHT_ANGLE + Turret.visionPitch!!).tan).inches).inches

    // motor controlling top roller speed
    val topShooter = KSparkMax(0).apply {
        kP = 10.0
        kD = 2.0
    }

    init {
        defaultCommand = Shoot
    }

    override fun periodic() {
        debugDashboard()
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "flywheel" to flywheelMaster,
            "top Shooter" to topShooter
        )
    }
}
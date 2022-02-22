package frc.robot.subsystems

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.shooter.Shoot
import frc.robot.subsystems.Turret.targetVisible
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.mechanisms.Flywheel
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.motorcontrol.KServo
import frc.kyberlib.motorcontrol.KSimulatedESC
import frc.kyberlib.simulation.Simulatable


/**
 * Current status of the shooter mechanism
 */
enum class SHOOTER_STATUS {
    IDLE, SPINUP, LOW_READY, HIGH_READY, AUTO_SHOT, FORCE_SHOT
}


/**
 * Encapsulates all the things relevant to shooting the ball
 */
object Shooter : SubsystemBase(), Debug, Simulatable {
    init {
        println("Intake")
    }
    var status = SHOOTER_STATUS.IDLE

    // main motor attached to the flywheel
    val flywheelMaster = KSimulatedESC(31).apply {
        identifier = "flywheel"
        radius = Constants.FLYWHEEL_RADIUS
        Notifier{this.velocity = this.velocitySetpoint}.startPeriodic(.002)
    }
    val flywheelControl = Flywheel(flywheelMaster, Constants.FLYWHEEL_MOMENT_OF_INERTIA, 4)
    // additional motors that copy the main
    private val flywheel2 = KSimulatedESC(32).apply { follow(flywheelMaster) }

    // Servo that sets the hood angle
    private val hood = KSimulatedESC(1)

    // FIXME: KServo is completely bs
    var hoodAngle: Angle
        get() = hood.position
        set(value) {
            hood.position = value
        }

    // how far from the hub the robot is (based on limelight)
    private val distanceFilter = LinearFilter.movingAverage(8)
    val targetDistance: Length? 
        get() = if (Game.sim) RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION).meters
                else if (!targetVisible) null
                else distanceFilter.calculate(((Constants.UPPER_HUB_HEIGHT - Constants.LIMELIGHT_HEIGHT) / (Constants.LIMELIGHT_ANGLE + Turret.visionPitch!!).tan).inches).inches

    // motor controlling top roller speed
    val topShooter = KSimulatedESC(33).apply {
        kP = 10.0
        kD = 2.0
    }
    private val topFollower = KSimulatedESC(34).apply {
        follow(topShooter)
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
            "top Shooter" to topShooter,
            "hood" to hood,
            "distance" to targetDistance
        )
    }

    override fun simUpdate(dt: Time) {
        flywheelControl.simUpdate(dt)
        // flywheelMaster.simVelocity = flywheelMaster.velocitySetpoint
        hood.simPosition = hood.positionSetpoint
        topShooter.simVelocity = topShooter.velocitySetpoint
    }
}
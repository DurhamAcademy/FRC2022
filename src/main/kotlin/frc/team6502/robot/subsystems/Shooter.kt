package frc.team6502.robot.subsystems

import edu.wpi.first.wpilibj.LinearFilter
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team6502.robot.Constants
import frc.team6502.robot.RobotContainer
import frc.team6502.robot.commands.shooter.Shoot
import frc.team6502.robot.subsystems.Turret.targetLost
import kyberlib.command.Debug
import kyberlib.command.Game
import kyberlib.math.units.extensions.*
import kyberlib.mechanisms.Flywheel
import kyberlib.motorcontrol.rev.KSparkMax


enum class SHOOTER_STATUS {
    IDLE, SPINUP, LOW_READY, HIGH_READY, AUTO_SHOT, FORCE_SHOT
}


object Shooter : SubsystemBase(), Debug {
    var status = SHOOTER_STATUS.IDLE

    val flywheelMaster = KSparkMax(0).apply {
        identifier = "flywheel"
        radius = Constants.FLYWHEEL_RADIUS
        Notifier{this.velocity = this.velocitySetpoint}.startPeriodic(.002)  // todo: test this
    }
    val flywheelControl = Flywheel(flywheelMaster, Constants.FLYWHEEL_MOMENT_OF_INERTIA, 4)
    private val flywheel2 = KSparkMax(0).apply { follow(flywheelMaster) }
    private val flywheel3 = KSparkMax(0).apply { follow(flywheelMaster) }
    private val flywheel4 = KSparkMax(0).apply { follow(flywheelMaster) }

    private val hood = Servo(1)

    // todo: figure out this will work - it won't figure it out
    var hoodAngle: Angle
        get() = hood.get().degrees
        set(value) {
            hood.set(value.degrees)
        }

    private val distanceFilter = LinearFilter.movingAverage(8)
    val targetDistance: Length? = if (Game.sim) RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION).meters
                                else if (targetLost) null
                                else distanceFilter.calculate(((Constants.UPPER_HUB_HEIGHT - Constants.LIMELIGHT_HEIGHT) / (Constants.LIMELIGHT_ANGLE + Turret.visionPitch).tan).inches).inches

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
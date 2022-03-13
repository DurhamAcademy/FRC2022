package frc.robot.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.math.filters.Differentiator
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.KSimulatedESC
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.servo.KLinearActuator
import frc.kyberlib.simulation.Simulatable
import frc.kyberlib.simulation.Simulation
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.subsystems.Turret.targetVisible


/**
 * Current status of the shooter mechanism
 */
enum class ShooterStatus {
    IDLE, SPINUP, SHOT, FORCE_SHOT
}


/**
 * Encapsulates all the things relevant to shooting the ball
 */
object Shooter : SubsystemBase(), Debug, Simulatable {
    var status = ShooterStatus.IDLE
    val ff = SimpleMotorFeedforward(0.99858, 0.18517, 0.0256)

    // main motor attached to the flywheel
    val flywheelMaster = KSparkMax(31).apply {
        identifier = "flywheel"
//        radius = Constants.FLYWHEEL_RADIUS
        motorType = DCMotor.getNEO(2)
        addFeedforward(ff)
        currentLimit = 50
    }

    var outputVelocity
        get() = flywheelMaster.velocity.toTangentialVelocity(Constants.BALL_DIAMETER - Constants.SHOOTER_COMPRESSION)
        set(value) {
            flywheelMaster.velocity = value.toAngularVelocity(Constants.BALL_DIAMETER - Constants.SHOOTER_COMPRESSION)
        }
    var timeOfFlight = 1.seconds

    var targetVelocity = 0.0.rpm


    // additional motors that copy the main
    private val flywheel2 = KSparkMax(32).apply {
        identifier = "flywheel2"
        reversed = true
        currentLimit = 50
        follow(flywheelMaster)
    }

    // Servo that sets the hood angle
    val hood = KLinearActuator(1, 100, 18.0)
    val hood2 = KLinearActuator(2, 100, 18.0)

//    val flywheelDiff = Differentiator()
    val flywheelPID = PIDController(0.1058, 0.0, 0.0)

    private fun controlLoop() {

        val actualVelocity = flywheelMaster.velocity.rpm / 60

        val pidOut = flywheelPID.calculate(actualVelocity, targetVelocity.rpm / 60)
        val ffOut = ff.calculate(targetVelocity.rpm / 60)
        flywheelMaster.voltage = pidOut + ffOut



        SmartDashboard.putNumber("ERROR RPM", flywheelPID.positionError * 60)
        SmartDashboard.putNumber("TARGET", targetVelocity.rpm)
        SmartDashboard.putNumber("ACTUAL", flywheelMaster.velocity.rpm)
        SmartDashboard.putNumber("PID", pidOut)
        SmartDashboard.putNumber("FF", ffOut)
    }

    var hoodDistance: Length
        get() = (hood.estimatedPosition * 10).centimeters
        set(value) {
            hood.targetPosition = value.centimeters * 10
            hood2.targetPosition = value.centimeters * 10
        }

    // how far from the center of the hub the robot is (based on limelight)
    private val distanceFilter = LinearFilter.movingAverage(8)
    val targetDistance: Length? 
        get() = if (Game.sim) RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION).meters
                else if (!targetVisible) null
                else 2.feet + distanceFilter.calculate((
                (Constants.UPPER_HUB_HEIGHT - 1.inches - Constants.LIMELIGHT_HEIGHT) / (Constants.LIMELIGHT_ANGLE + Turret.visionPitch!!).tan).inches).inches

    init {
        if(Game.sim) Simulation.instance.include(this)
    }

    override fun periodic() {
        debugDashboard()
        controlLoop()
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "flywheel" to flywheelMaster,
            "hood" to hoodDistance,
            "distance" to targetDistance
        )
    }

    override fun simUpdate(dt: Time) {
//        flywheelControl.simUpdate(dt)
//        hood.simPosition = hood.positionSetpoint
//        topShooter.simVelocity = topShooter.velocitySetpoint
    }
}
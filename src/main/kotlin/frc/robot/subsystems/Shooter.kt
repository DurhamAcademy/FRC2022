package frc.robot.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.math.randomizer
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.servo.KLinearActuator
import frc.kyberlib.simulation.Simulatable
import frc.kyberlib.simulation.Simulation
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.subsystems.Turret.targetVisible
import kotlin.math.acos
import kotlin.math.cos
import kotlin.math.sqrt


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
        kP = 0.1058
        currentLimit = 50
    }

//    var outputVelocity
//        get() = flywheelMaster.velocity.toTangentialVelocity(Constants.BALL_DIAMETER - Constants.SHOOTER_COMPRESSION)
//        set(value) {
//            flywheelMaster.velocity = value.toAngularVelocity(Constants.BALL_DIAMETER - Constants.SHOOTER_COMPRESSION)
//        }
//    var timeOfFlight = 1.seconds
//
    var targetVelocity
        get() = flywheelMaster.velocitySetpoint
        set(value) { flywheelMaster.velocity = targetVelocity}


    // additional motors that copy the main
    private val flywheel2 = KSparkMax(32).apply {
        identifier = "flywheel2"
        reversed = true
        currentLimit = 50
        follow(flywheelMaster)
    }

    // Servo that sets the hood angle
    val hood = KLinearActuator(1, 100, 18.0)
    private val hood2 = KLinearActuator(2, 100, 18.0)

    private var hoodDistance: Length
        get() = hood.position
        set(value) {
            hood.position = value
            hood2.position = value
        }

    //https://www.mathsisfun.com/algebra/trig-cosine-law.html
    // c2 = a2 + b2 − 2ab cos(C)
    private const val A = 6.085  // flywheel axis to servo axis
    private const val A2 = A*A
    private const val B = 10.016  // flywheel axis to servo end
    private const val B2 = B*B
    private const val D = 2 * A * B
    private const val theta = 0.0
    var hoodAngle: Angle
        get() = acos((-hoodDistance.inches * hoodDistance.inches + A2 + B2)/D).radians
        set(value) {
            hoodDistance = sqrt(A2 + B2 - D * cos(theta + value.radians)).inches
        }

    // how far from the center of the hub the robot is (based on limelight)
    private val distanceFilter = LinearFilter.movingAverage(8)
    val targetDistance: Length? 
        get() = if (Game.sim) RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION).meters + randomizer.nextGaussian().inches
                else if (!targetVisible) null
                else 2.feet + distanceFilter.calculate((
                (Constants.UPPER_HUB_HEIGHT - 1.inches - Constants.LIMELIGHT_HEIGHT) / (Constants.LIMELIGHT_ANGLE + Turret.visionPitch!!).tan).inches).inches

    init {
        if(Game.sim) Simulation.instance.include(this)
    }

    override fun periodic() {
        debugDashboard()
        flywheelMaster.updateVoltage()
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
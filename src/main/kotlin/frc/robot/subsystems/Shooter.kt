package frc.robot.subsystems

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.Debouncer.DebounceType
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.math.randomizer
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.motorcontrol.servo.KLinearServo
import frc.kyberlib.simulation.Simulatable
import frc.kyberlib.simulation.Simulation
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.shooter.Shoot
import kotlin.math.acos
import kotlin.math.cos
import kotlin.math.sqrt


/**
 * Encapsulates all the things relevant to shooting the ball
 */
object Shooter : SubsystemBase(), Debug, Simulatable {
    var status = ShooterStatus.IDLE
//    override val priority: DebugFilter = DebugFilter.Max
    private val ff = SimpleMotorFeedforward(0.99858 / TAU, 0.19517 / TAU, 0.0256 / TAU)

    // main motor attached to the flywheel
    val flywheel = KSparkMax(31).apply {
        identifier = "flywheel"
        motorType = DCMotor.getNEO(2)
        addFeedforward(ff)
        kP = 0.258 / TAU
        currentLimit = 50
        if(Game.sim) setupSim(ff)
    }

    private val readyDebouncer = Debouncer(0.2, DebounceType.kBoth)
    private val shortReady
        get() = hood.atSetpoint && flywheel.velocity < Constants.SHOOTER_VELOCITY_TOLERANCE && flywheel.velocitySetpoint > 1.radiansPerSecond
    val ready: Boolean
        get() = readyDebouncer.calculate(shortReady)
    val stopped
        get() = flywheel.percent == 0.0

//    var outputVelocity
//        get() = flywheel.velocity.toTangentialVelocity(Constants.BALL_DIAMETER - Constants.SHOOTER_COMPRESSION)
//        set(value) {
//            flywheel.velocity = value.toAngularVelocity(Constants.BALL_DIAMETER - Constants.SHOOTER_COMPRESSION)
//        }
//    var timeOfFlight = 1.seconds
//
    var shooterMult = 1.0
    var targetVelocity
        get() = flywheel.velocitySetpoint
        set(value) { flywheel.velocity = value * shooterMult}


    // additional motors that copy the main
    private val flywheel2 = KSparkMax(32).apply {
        identifier = "flywheel2"
        reversed = true
        currentLimit = 50
        follow(flywheel)
    }

    // Servo that sets the hood angle
    private val hood = KLinearServo(8, 100, 18.0.millimetersPerSecond)
    private val hood2 = KLinearServo(9, 100, 18.0.millimetersPerSecond)

    var hoodDistance: Length
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
    private val theta = 24.5476.degrees.radians
    private val startLength = 6.61.inches
    var hoodAngle: Angle
        get() = acos((-hoodDistance.inches * hoodDistance.inches + A2 + B2)/D).radians
        set(value) {
            hoodDistance = sqrt(A2 + B2 - D * cos(theta + value.radians)).inches - startLength
        }

    fun update() {
        val dis = Turret.targetDistance ?: RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION).meters
        flywheelUpdate(dis)
        hoodUpdate(dis)
    }
    private fun hoodUpdate(dis: Length) {
        hoodDistance = Constants.HOODANGLE_INTERPOLATOR.calculate(dis.meters).millimeters
    }
    private fun flywheelUpdate(dis: Length) {
        targetVelocity = Constants.FLYWHEEL_INTERPOLATOR.calculate(dis.meters).rpm
    }

    fun stop() {
        targetVelocity = 0.rpm
        flywheel.stop()
        status = ShooterStatus.IDLE
    }

    init {
        if(Game.sim) Simulation.instance.include(this)
    }

    override fun periodic() {
        debugDashboard()
        Turret.targetDistance?.let { hoodUpdate(it) }
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "flywheel" to flywheel,
            "hood" to hood,
            "distance" to Turret.targetDistance
        )
    }

    override fun simUpdate(dt: Time) {
//        flywheelControl.simUpdate(dt)
//        hood.simPosition = hood.positionSetpoint
//        topShooter.simVelocity = topShooter.velocitySetpoint
    }
}
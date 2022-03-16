package frc.robot.subsystems

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.Debouncer.DebounceType
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.command.Game
import frc.kyberlib.math.Polynomial
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
 * Current status of the shooter mechanism
 */
enum class ShooterStatus {
    IDLE, SPINUP, LOW_READY, HIGH_READY, SHOT, FORCE_SHOT
}

/**
 * Encapsulates all the things relevant to shooting the ball
 */
object Shooter : SubsystemBase(), Debug, Simulatable {
    var status = ShooterStatus.IDLE
//    override val priority: DebugFilter = DebugFilter.Max
    private val ff = SimpleMotorFeedforward(0.2062, 0.0315032, 0.0052967)

    // main motor attached to the flywheel
    val flywheel = KSparkMax(31).apply {
        identifier = "flywheel"
        motorType = DCMotor.getNEO(2)
        addFeedforward(ff)
        kP = 0.032434
//        kI = 0.001
        currentLimit = 40
        brakeMode = false
        if(Game.sim) setupSim(ff)
    }

    private val readyDebouncer = Debouncer(0.2, DebounceType.kBoth)
    private val shortReady
        get() = hood.atSetpoint && flywheel.velocityError.absoluteValue < Constants.SHOOTER_VELOCITY_TOLERANCE && flywheel.velocitySetpoint > 1.radiansPerSecond
    val ready: Boolean
        get() = readyDebouncer.calculate(shortReady)
    val stopped
        get() = flywheel.percent == 0.0

    var targetVelocity
        get() = flywheel.velocitySetpoint
        set(value) { flywheel.velocity = value * SmartDashboard.getNumber("shooterMult", 1.0)}


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

    val hoodPoly = Polynomial(-.85458, 5.64695, 3.87906, -1.29395)
    private fun hoodUpdate(dis: Length) {
        hoodDistance = hoodPoly.eval(dis.value).millimeters//Constants.HOODANGLE_INTERPOLATOR.calculate(dis.meters).millimeters
    }

    val speedPoly = Polynomial(37.43917, -119.05297, 1501.93519)
    private fun flywheelUpdate(dis: Length) {
        val interpolated = speedPoly.eval(dis.value).rpm//.coerceAtMost(2000.rpm)//Constants.FLYWHEEL_INTERPOLATOR.calculate(dis.meters).rpm
        val fudge = SmartDashboard.getNumber("back fudge", 100.0).rpm * (Turret.turret.position.absoluteValue / 2.0).sin
        targetVelocity = (interpolated + fudge) / 2.0
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
//        debugDashboard()
        SmartDashboard.putNumber("fly error", flywheel.velocityError.rpm)
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
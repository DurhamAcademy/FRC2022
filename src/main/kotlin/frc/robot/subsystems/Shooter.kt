package frc.robot.subsystems

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.Debouncer.DebounceType
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.command.Game
import frc.kyberlib.math.Polynomial
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.ctre.KTalon
import frc.kyberlib.motorcontrol.servo.KLinearServo
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.shooter.ShooterCalibration
import kotlin.math.*


/**
 * Current status of the shooter mechanism
 */
enum class ShooterStatus {
    IDLE, SPINUP, SHOT, FORCE_SHOT
}

/**
 * Encapsulates all the things relevant to shooting the ball
 */
object Shooter : SubsystemBase(), Debug {
    var inRange: Boolean = false
        private set

    var status = ShooterStatus.IDLE
    override val priority: DebugFilter = DebugFilter.Max
    private val ff = SimpleMotorFeedforward(0.81436, 0.01918, 0.0045666)
    var time = Game.time

    // main motor attached to the flywheel
    val flywheel = KTalon(32).apply {
        // configs
        identifier = "flywheel"
        motorType = DCMotor.getFalcon500(2)
        currentLimit = 50
        brakeMode = false
        addFeedforward(ff)
        kP = 0.20569
        setupSim()
//        setupSim(flywheelSystem(Constants.FLYWHEEL_MOMENT_OF_INERTIA))
    }

    // smooth out when shooter up to speed
    private val readyDebouncer = Debouncer(0.2, DebounceType.kBoth)  // fixme: maybe just kFalling
    private val shortReady
        inline get() = hood.atSetpoint &&
                    flywheel.velocityError.absoluteValue < (if (RobotContainer.op.shootWhileMoving) 50.rpm else Constants.SHOOTER_VELOCITY_TOLERANCE)
                    && flywheel.velocitySetpoint > 1.radiansPerSecond
    val ready: Boolean  // whether ready to shoot
        get() = readyDebouncer.calculate(shortReady)

    var targetVelocity  // public accesser for setting flywheel speed
        inline get() = flywheel.velocitySetpoint
        inline set(value) {
            SmartDashboard.putNumber("flywheel target", value.rpm)
            flywheel.velocity = value * SmartDashboard.getNumber("shooterMult", 1.0)
        }

    // additional motors that copy the main
    private val flywheel2 = KTalon(31).apply {
        identifier = "flywheel2"
        reversed = true
        brakeMode = false
        currentLimit = 50
        follow(flywheel)
    }

    // Servo that sets the hood angle
    private val hood = KLinearServo(5, 100.millimeters, 18.0.millimetersPerSecond)
    private val hood2 = KLinearServo(6, 100.millimeters, 18.0.millimetersPerSecond)

    var hoodDistance: Length  // public accessor var. Makes them move in sync
        get() = hood.position
        set(value) {
            SmartDashboard.putNumber("hood dis", value.millimeters)
            hood.position = value
            hood2.position = value
        }

    //https://www.mathsisfun.com/algebra/trig-cosine-law.html
    // c2 = a2 + b2 − 2ab cos(C)
    private const val A = 6.085  // flywheel axis to servo axis
    private const val A2 = A * A
    private const val B = 10.016  // flywheel axis to servo end
    private const val B2 = B * B
    private const val D = 2 * A * B
    private val theta = 24.258.degrees.radians
    private val startLength = 6.61.inches
    var hoodAngle: Angle  // set the angle of the hood. Not applied through most of the code because only got working later
        get() = acos((-hoodDistance.inches * hoodDistance.inches + A2 + B2) / D).radians
        set(value) {
            hoodDistance = sqrt(A2 + B2 - D * cos(theta + value.radians)).inches - startLength
        }

    // update shooter stuff
    fun update() {
        flywheelUpdate(Limelight.effectiveDistance)
        hoodUpdate(Limelight.effectiveDistance)
    }

    private val hoodPoly = Polynomial(-.85458, 5.64695, 3.87906, -1.29395, domain = 1.7..5.5)  // poly fitted through our data

    fun hoodUpdate(dis: Length) {  // update the hood angle with a certain distance
        val hood = Constants.HOODANGLE_INTERPOLATOR.calculate(dis.meters)// hoodPoly.eval(dis.value)
        inRange = dis < 5.8.meters
        hoodDistance = hood.millimeters
    }

    fun flywheelUpdate(dis: Length) {  // update flywheel speed to shoot certain distance
        val interpolated = Constants.FLYWHEEL_INTERPOLATOR.calculate(dis.meters)

        val fudge = 1 + (SmartDashboard.getNumber("back fudge", 0.03)) *
                        (Turret.turret.position / 2.0).sin.absoluteValue
        targetVelocity = interpolated.rpm * fudge
    }

    fun stop() {  // stop the shooter
        targetVelocity = 0.rpm
        flywheel.stop()
        status = ShooterStatus.IDLE
    }

    override fun periodic() {
        debugDashboard()
        // log stuff
        SmartDashboard.putNumber("fly error", flywheel.velocityError.rpm)
        SmartDashboard.putNumber("rpm", flywheel.velocity.rpm)
        if (currentCommand != ShooterCalibration) hoodUpdate(Limelight.effectiveDistance)
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "flywheel" to flywheel,
            "hood" to hood
        )
    }
}
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
    var status = ShooterStatus.IDLE
    override val priority: DebugFilter = DebugFilter.Max
    private val ff = SimpleMotorFeedforward(0.81436, 0.01918, 0.0045666)  // fixme

    // main motor attached to the flywheel
    private val flywheel = KTalon(32).apply {
        // configs
        identifier = "flywheel"
        motorType = DCMotor.getFalcon500(2)
        currentLimit = 50
        brakeMode = false
        nativeControl = true
        talon.config_kP(0, 0.02569)
//        kP = 0.05//20569
        addFeedforward(ff)
        setupSim()
//        setupSim(flywheelSystem(Constants.FLYWHEEL_MOMENT_OF_INERTIA))
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

    var targetVelocity  // public accesser for setting flywheel speed
        get() = flywheel.velocitySetpoint
        set(value) {
            SmartDashboard.putNumber("flywheel target", value.rpm)
            flywheel.velocity = value * SmartDashboard.getNumber("shooterMult", 1.0)
        }

    var hoodDistance: Length  // public accessor var. Makes them move in sync
        get() = hood.position
        set(value) {
            SmartDashboard.putNumber("hood dis", value.millimeters)
            hood.position = value
            hood2.position = value
        }

    // update shooter stuff
    fun update(dis: Length = Limelight.effectiveDistance) {
        flywheelUpdate(dis)
        hoodUpdate(dis)
    }

    private fun hoodUpdate(dis: Length) {  // update the hood angle with a certain distance
        val hood = Constants.HOODANGLE_INTERPOLATOR.calculate(dis.meters)// hoodPoly.eval(dis.value)
        inRange = dis < 5.8.meters
        hoodDistance = hood.millimeters
    }

    private fun flywheelUpdate(dis: Length) {  // update flywheel speed to shoot certain distance
        val interpolated = Constants.FLYWHEEL_INTERPOLATOR.calculate(dis.meters)

        val fudge = 1 + (SmartDashboard.getNumber("back fudge", 0.03)) *
                        (Turret.position / 2.0).sin.absoluteValue
        targetVelocity = interpolated.rpm * fudge
    }

    fun stop() {  // stop the shooter
        targetVelocity = 0.rpm
        flywheel.stop()
        status = ShooterStatus.IDLE
    }

    fun yeet() {
        flywheel.percent = 0.9
    }

    // smooth out when shooter up to speed
    private val readyDebouncer = Debouncer(0.2, DebounceType.kBoth)
    private val shortReady
        inline get() = hood.atSetpoint &&
                flywheel.velocityError.absoluteValue < (if (RobotContainer.op.shootWhileMoving) 50.rpm else Constants.SHOOTER_VELOCITY_TOLERANCE)
                && flywheel.velocitySetpoint > 1.radiansPerSecond
    val ready: Boolean  // whether ready to shoot
        get() = readyDebouncer.calculate(shortReady)
    var inRange = false
        private set

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
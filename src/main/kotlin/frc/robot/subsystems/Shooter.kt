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
object Shooter : SubsystemBase() {
    var inRange: Boolean = false
        private set

    var status = ShooterStatus.IDLE
    private val ff = SimpleMotorFeedforward(0.81436, 0.01918, 0.0045666)
    var time = Game.time

    // main motor attached to the flywheel
    val flywheel = KTalon(32).apply {
        // configs
        identifier = "flywheel"
        motorType = DCMotor.getFalcon500(2)
        currentLimit = 50
        brakeMode = false
        addFeedforward(ff) // todo
        kP = 0.20569
        setupSim()
//        setupSim(flywheelSystem(Constants.FLYWHEEL_MOMENT_OF_INERTIA))
    }

    // smooth out when shooter up to speed
    private val readyDebouncer = Debouncer(0.2, DebounceType.kBoth)
    private val shortReady
        inline get() = hood.positionError.absoluteValue < 2.degrees &&
                    flywheel.velocityError.absoluteValue < Constants.SHOOTER_VELOCITY_TOLERANCE
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
    val hood = KTalon(-1)

    var hoodAngle: Angle  // public accessor var. Makes them move in sync
        get() = hood.position
        set(value) {
            hood.position = value
        }

    // update shooter stuff
    fun update() {  // todo
        flywheelUpdate(Limelight.effectiveDistance)
        hoodUpdate(Limelight.effectiveDistance)
    }

    fun hoodUpdate(dis: Length) {  // update the hood angle with a certain distance
        val hood = Constants.HOODANGLE_INTERPOLATOR.calculate(dis.meters)// hoodPoly.eval(dis.value)
        inRange = dis < 5.8.meters
        hoodAngle = hood.degrees
    }

    fun flywheelUpdate(dis: Length) {  // update flywheel speed to shoot certain distance
        val interpolated = Constants.FLYWHEEL_INTERPOLATOR.calculate(dis.meters)
        targetVelocity = interpolated.rpm
    }

    fun stop() {  // stop the shooter
        targetVelocity = 0.rpm
        flywheel.stop()
        status = ShooterStatus.IDLE
    }

    override fun periodic() {
        // log stuff
        SmartDashboard.putNumber("fly error", flywheel.velocityError.rpm)
        SmartDashboard.putNumber("rpm", flywheel.velocity.rpm)
        if (currentCommand != ShooterCalibration) hoodUpdate(Limelight.effectiveDistance)
    }
}
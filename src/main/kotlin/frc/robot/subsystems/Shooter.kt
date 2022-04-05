package frc.robot.subsystems

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.Debouncer.DebounceType
import edu.wpi.first.math.geometry.Translation2d
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
import frc.kyberlib.simulation.Simulatable
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.shooter.FireWhenReady
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
object Shooter : SubsystemBase(), Debug, Simulatable {
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

        arbFF = {
            ff.calculate(velocitySetpoint.radiansPerSecond)
        }

        talon.config_kP(0, 0.20569)

        if (Game.sim) setupSim(ff)
    }

    // smooth out when shooter up to speed
    private val readyDebouncer = Debouncer(0.2, DebounceType.kBoth)  // fixme: maybe just kFalling
    private val shortReady
        inline get() = hood.atSetpoint &&
                flywheel.velocityError.absoluteValue < (if (RobotContainer.op.shootWhileMoving) 50.rpm else Constants.SHOOTER_VELOCITY_TOLERANCE)
                && flywheel.velocitySetpoint > 1.radiansPerSecond
    val ready: Boolean  // whether ready to shoot
        get() = readyDebouncer.calculate(shortReady)
    val stopped
        get() = flywheel.percent == 0.0

    var targetVelocity  // public accesser for setting flywheel speed
        inline get() = flywheel.velocitySetpoint
        inline set(value) {
            SmartDashboard.putNumber("flywheel target", value.rpm)
            flywheel.velocity = value * SmartDashboard.getNumber("shooterMult", 1.0)
        }

    // distance function to work in several scenarios
    private val smartDis: Length
        get() {
            return if (RobotContainer.op.shootWhileMoving) {
                effectiveDistance
            } else {
                Turret.targetDistance ?: RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION).meters
            }
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
    private val hood = KLinearServo(8, 100.millimeters, 18.0.millimetersPerSecond)
    private val hood2 = KLinearServo(9, 100.millimeters, 18.0.millimetersPerSecond)

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
        flywheelUpdate(smartDis)
        hoodUpdate(smartDis)
    }

    private const val moveIterations = 1  // how many times to optimize the shoot while move target
    val effectiveHubLocation: Translation2d  // translated hub position based on robot velocity
        get() {
            val base = Constants.HUB_POSITION
            if (!Constants.MOVEMENT_CORRECTION) return base
            val fieldSpeeds = Drivetrain.fieldRelativeSpeeds
            return base - Translation2d(
                fieldSpeeds.vxMetersPerSecond * timeOfFlight.value,
                fieldSpeeds.vyMetersPerSecond * timeOfFlight.value
            )
        }
    var movementAngleOffset: Angle = 0.degrees  // how much robot movement should cause turret to turn to compensate
    var effectiveDistance: Length = 0.meters  // how far shoooter should behave to compensate for movement

    private val timeOfFlight  // how long it should take the ball to score
        get() = Constants.TIME_OF_FLIGHT_INTERPOLATOR.calculate(smartDis.meters).seconds

    private val hoodPoly =
        Polynomial(-.85458, 5.64695, 3.87906, -1.29395, domain = 1.7..5.5)  // poly fitted through our data

    fun hoodUpdate(dis: Length) {  // update the hood angle with a certain distance
        val hood = hoodPoly.eval(dis.value)
        if (hood == null) {
            inRange = false
        } else {
            inRange = true
            hoodDistance = hood.millimeters//Constants.HOODANGLE_INTERPOLATOR.calculate(dis.meters).millimeters
        }
    }

    private val speedPoly = Polynomial(37.43917, -119.05297, 1501.93519)
    fun flywheelUpdate(dis: Length) {  // update flywheel speed to shoot certain distance
        val interpolated =
            speedPoly.eval(dis.value) //.coerceAtMost(2000.rpm)//Constants.FLYWHEEL_INTERPOLATOR.calculate(dis.meters).rpm

        if (interpolated != null) {
            val fudge = 1 + (SmartDashboard.getNumber(
                "back fudge",
                0.03
            )) * (Turret.turret.position / 2.0).sin.absoluteValue
            targetVelocity = interpolated.rpm * fudge
        }
    }

    fun stop() {  // stop the shooter
        targetVelocity = 0.rpm
        flywheel.stop()
        status = ShooterStatus.IDLE
    }

    init {
        // setup stuff
        if (Game.sim) flywheel.setupSim(ff)
        if (RobotContainer.op.autoShot) defaultCommand = FireWhenReady
    }

    override fun periodic() {
        debugDashboard()
        if (RobotContainer.op.shootWhileMoving) {
            // update shooting while moving values
            val rawDistance =
                Turret.targetDistance ?: RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION).meters
            var r = rawDistance
            val hubSpeeds = Drivetrain.hubRelativeSpeeds
            val parallel = hubSpeeds.vxMetersPerSecond
            val perp = hubSpeeds.vyMetersPerSecond
            for (i in 0 until moveIterations) {
                val time = Constants.TIME_OF_FLIGHT_INTERPOLATOR.calculate(r.meters) * SmartDashboard.getNumber(
                    "time mult",
                    1.0
                )
                val a = r.meters - parallel * time
                val b = perp * time
                r = sqrt(a.pow(2) + b.pow(2)).meters.absoluteValue
                movementAngleOffset = atan(b / a).radians * .3
            }
            SmartDashboard.putNumber("effective distance", r.meters)
            SmartDashboard.putNumber("effective turr offset", movementAngleOffset.degrees)
            SmartDashboard.putNumber("parallel", parallel)
            SmartDashboard.putNumber("perp", perp)
            effectiveDistance = r
        }
        // log stuff
        SmartDashboard.putNumber("fly error", flywheel.velocityError.rpm)
        SmartDashboard.putNumber("rpm", flywheel.velocity.rpm)
        if (currentCommand != ShooterCalibration) hoodUpdate(smartDis)
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "flywheel" to flywheel,
            "hood" to hood,
            "distance" to smartDis
        )
    }

    override fun simUpdate(dt: Time) {

    }
}
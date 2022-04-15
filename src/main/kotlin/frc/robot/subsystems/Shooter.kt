package frc.robot.subsystems

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.ctre.KTalon
import frc.kyberlib.motorcontrol.servo.KLinearServo
import frc.robot.Constants
import kotlin.math.*

/**
 * Encapsulates all the things relevant to shooting the ball
 */
object Shooter : SubsystemBase() {
    private val ff = SimpleMotorFeedforward(0.81436, 0.01918, 0.0045666)

    // main motor attached to the flywheel
    val flywheel = KTalon(32).apply {
        currentLimit = 50
        brakeMode = false
        addFeedforward(ff)
        kP = 0.20569
        setupSim()
//        setupSim(flywheelSystem(Constants.FLYWHEEL_MOMENT_OF_INERTIA))
    }

    // smooth out when shooter up to speed
    val ready
        get() = hood.atSetpoint &&
                flywheel.velocityError.absoluteValue < Constants.SHOOTER_VELOCITY_TOLERANCE &&
                flywheel.velocitySetpoint > 1.radiansPerSecond

    var targetVelocity  // public accesser for setting flywheel speed
        inline get() = flywheel.velocitySetpoint
        inline set(value) { flywheel.velocity = value }

    // additional motors that copy the main
    private val flywheel2 = KTalon(31).apply {
        copyConfig(flywheel)
        follow(flywheel)
    }

    // Servo that sets the hood angle
    private val hood = KLinearServo(5, 100.millimeters, 18.0.millimetersPerSecond)
    private val hood2 = KLinearServo(6, 100.millimeters, 18.0.millimetersPerSecond)

    var hoodDistance: Length  // public accessor var. Makes them move in sync
        get() = hood.position
        set(value) {
            hood.position = value
            hood2.position = value
        }
    // update shooter stuff
    fun update() {
        targetVelocity = Constants.FLYWHEEL_INTERPOLATOR.calculate(Limelight.distance.meters).rpm
        hoodDistance = Constants.HOODANGLE_INTERPOLATOR.calculate(Limelight.distance.meters).millimeters
    }
    fun hoodUpdate(dis: Length) {  // update the hood angle with a certain distance
        hoodDistance = Constants.HOODANGLE_INTERPOLATOR.calculate(dis.meters).millimeters
    }
    fun stop() {  // stop the shooter
        targetVelocity = 0.rpm
        flywheel.stop()
    }

    override fun periodic() {
        hoodUpdate(Limelight.distance)
    }
}
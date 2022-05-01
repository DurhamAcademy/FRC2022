package frc.kyberlib.motorcontrol.servo

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.Servo
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.Angle
import frc.kyberlib.math.units.extensions.AngularVelocity
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.radians


/**
 * Handles a angle servo
 * @param port the id the servo is plugging into the rio
 * @param length the max readable value (angle)
 * @param travelSpeed the estimated angularVelocity that the server goes. Used for angle estimation
 * @param initialPosition where the servo starts upon enabled. Used for angle estimation
 * @param minMs the minimum milliseconds that the servo reads as valid signal
 * @param maxMs the maximum milliseconds that the servo reads as valid signal
 */
class KServo(
    port: Int,
    val length: Int,
    travelSpeed: AngularVelocity,
    initialPosition: Angle = 0.degrees,
    minMs: Double = 1.0,
    maxMs: Double = 2.0
) : Debug {
    /**
     * The actual servo native object
     */
    private val servo = Servo(port).apply {
        // set PWM bounds of the servo
        setBounds(maxMs, 0.01 + (maxMs + minMs) / 2, (maxMs + minMs) / 2, -0.01 + (maxMs + minMs) / 2, minMs)
    }
    override var identifier: String = "Servo$port"

    // rate limiter for estimating current actuator angle
    private val rateLimit = SlewRateLimiter(travelSpeed.value, initialPosition.value)

    /**
     * The angle of the motor. In order for estimated to work, this should update frequently
     */
    var position: Angle = initialPosition
        set(value) {
            // command the servo to new target angle
            servo.position = value.degrees / length
            // update the angle estimate
            field = rateLimit.calculate(value.value).radians
            // update backing field
            setpoint = value
        }

    /**
     * Where the servo is trying to get to
     */
    var setpoint: Angle = initialPosition

    /**
     * Guess of how far off the servo is from setpoint
     */
    inline val error: Angle
        get() = position - setpoint

    /**
     * True if the actuator's estimated angle has converged on the target angle.
     */
    inline val atSetpoint: Boolean
        get() = error.absoluteValue.degrees < 2.0
}
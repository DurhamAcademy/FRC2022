package frc.kyberlib.servo

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.Servo
import kotlin.math.abs

/**
 * A PWM linear actuator.
 * @param port the PWM port the actuator is connected to
 * @param length the maximum extended length of the actuator, in mm
 * @param travelSpeed the speed of the actuator when in motion in mm/s (this may be slower than spec-sheet value due to load)
 * @param initialPosition the position this actuator will start in
 * @param minMs minimum length PWM pulse width, in ms
 * @param maxMs maximum length PWM pulse width, in ms
 */
class KLinearActuator(
    port: Int,
    val length: Int,
    travelSpeed: Double,
    initialPosition: Double = 0.0,
    minMs: Double = 1.0,
    maxMs: Double = 2.0
) {

    // the actual hardware servo being controlled
    private val servo = Servo(port).apply {
        // set PWM bounds of the servo
        setBounds(maxMs, 0.01 + (maxMs + minMs) / 2, (maxMs + minMs) / 2, -0.01 + (maxMs + minMs) / 2, minMs)
    }

    // rate limiter for estimating current actuator position
    private val rateLimit = SlewRateLimiter(travelSpeed, initialPosition)

    /**
     * Target actuator length, in mm
     */
    var targetPosition: Double = initialPosition
        set(value) {
            // command the servo to new target position
            servo.position = value / length
            // update the position estimate
            estimatedPosition = rateLimit.calculate(value)
            // update backing field
            field = value
        }

    /**
     * Estimate of where the actuator currently is, in mm
     */
    var estimatedPosition: Double = initialPosition
        private set

    /**
     * True if the actuator's estimated position has converged on the target position.
     */
    val atSetpoint: Boolean
        get() = abs(targetPosition - estimatedPosition) < 2.0
}
package frc.kyberlib.motorcontrol.servo

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.Servo
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.*

/**
 * A PWM linear actuator.
 * @param port the PWM port the actuator is connected to
 * @param length the maximum extended length of the actuator, in mm
 * @param travelSpeed the speed of the actuator when in motion in mm/s (this may be slower than spec-sheet value due to load)
 * @param initialPosition the position this actuator will start in
 * @param minMs minimum length PWM pulse width, in ms
 * @param maxMs maximum length PWM pulse width, in ms
 */
class KLinearServo(
    port: Int,
    val length: Length,
    travelSpeed: LinearVelocity,
    initialPosition: Length = 0.millimeters,
    minMs: Double = 1.0,
    maxMs: Double = 2.0
) : Debug {
    override val identifier: String = "Linear Servo $port"

    // the actual hardware servo being controlled
    private val servo = Servo(port).apply {
        // set PWM bounds of the servo
        setBounds(maxMs, 0.01 + (maxMs + minMs) / 2, (maxMs + minMs) / 2, -0.01 + (maxMs + minMs) / 2, minMs)
    }

    // rate limiter for estimating current actuator position
    private val rateLimit = SlewRateLimiter(travelSpeed.millimetersPerSecond, initialPosition.millimeters)

    /**
     * Target actuator length, in mm
     */
    var position: Length = initialPosition
        set(value) {
            // command the servo to new target position
            servo.position = value / length
            // update the position estimate
            field = rateLimit.calculate(value.millimeters).millimeters
            // update backing field
            setpoint = value
        }

    var setpoint: Length = initialPosition

    val error: Length
        get() = position - setpoint
    /**
     * True if the actuator's estimated position has converged on the target position.
     */
    val atSetpoint: Boolean
        get() = error.absoluteValue.millimeters < 2.0

    override fun debugValues(): Map<String, Any?> {
        return mapOf("position" to position.millimeters, "setpoint" to setpoint.millimeters)
    }
}
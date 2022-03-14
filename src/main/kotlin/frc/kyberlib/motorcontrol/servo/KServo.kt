package frc.kyberlib.motorcontrol.servo

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Servo
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.*
import java.lang.UnsupportedOperationException

// todo: add documentation
/**
 * Handles a position servo
 */
class KServo(port: Int,
             val length: Int,
             travelSpeed: AngularVelocity,
             initialPosition: Angle = 0.degrees,
             minMs: Double = 1.0,
             maxMs: Double = 2.0) : Debug {
    private val servo = Servo(port).apply {
        // set PWM bounds of the servo
        setBounds(maxMs, 0.01 + (maxMs + minMs) / 2, (maxMs + minMs) / 2, -0.01 + (maxMs + minMs) / 2, minMs)
    }
    override var identifier: String = "Servo$port"
    init {
        PWMRegristry[identifier] = port
    }

    // rate limiter for estimating current actuator position
    private val rateLimit = SlewRateLimiter(travelSpeed.value, initialPosition.value)


    /**
     * Target actuator length, in mm
     */
    var position: Angle = initialPosition
        set(value) {
            // command the servo to new target position
            servo.position = value.degrees / length
            // update the position estimate
            field = rateLimit.calculate(value.value).radians
            // update backing field
            setpoint = value
        }

    var setpoint: Angle = initialPosition

    val error: Angle
        get() = position - setpoint
    /**
     * True if the actuator's estimated position has converged on the target position.
     */
    val atSetpoint: Boolean
        get() = error.absoluteValue.degrees < 2.0

    override fun debugValues(): Map<String, Any?> {
        return mapOf("position" to position.degrees, "setpoint" to setpoint.degrees)
    }
}
package frc.kyberlib.motorcontrol

import edu.wpi.first.wpilibj.Servo
import frc.kyberlib.command.LogMode
import frc.kyberlib.math.units.extensions.Angle
import frc.kyberlib.math.units.extensions.AngularVelocity
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.radiansPerSecond
import kotlin.math.absoluteValue

class KServo(port: Int) : KMotorController() {
    private val native = Servo(port)
    private val absoluteAngle
        get() = native.angle.degrees

    private var accumulatedPosition = 0.degrees
    private var prevAbsoluteAngle = absoluteAngle

    override fun resetPosition(position: Angle) {
        accumulatedPosition = position
        prevAbsoluteAngle = absoluteAngle
    }

    override var rawPosition: Angle
        get() = accumulatedPosition
        set(value) {
            val positionChange = value - accumulatedPosition
            if (positionChange.degrees.absoluteValue < 179) native.angle = value.normalized.degrees
            else if (positionChange.degrees > 0) native.speed = 1.0

        }
    override var rawVelocity: AngularVelocity
        get() = native.speed.radiansPerSecond  // TODO: check if this is true
        set(value) {
            log("Don't velocity control servos", LogMode.WARN)
            native.speed = value.radiansPerSecond.coerceIn(-1.0, 1.0)
        }
    override var currentLimit: Int
        get() = 10
        set(value) {log("this no work on Servo", LogMode.WARN)}

    override fun writePid(p: Double, i: Double, d: Double) {
    }

    override fun writeMultipler(mv: Double, mp: Double) {
    }

    override fun configureEncoder(config: KEncoderConfig): Boolean {
        return true
    }

    override var brakeMode: BrakeMode
        get() = true
        set(value) {}
    override var rawReversed: Boolean
        get() = false
        set(value) {}
    override var identifier: String
        get() = "servo"
        set(value) {}
    override var rawPercent: Double
        get() = native.speed
        set(value) {native.speed = value}

    override fun followTarget(kmc: KBasicMotorController) {
        TODO("Not yet implemented")
    }
}
package frc.kyberlib.motorcontrol

import edu.wpi.first.wpilibj.Servo
import frc.kyberlib.command.LogMode
import frc.kyberlib.math.units.extensions.Angle
import frc.kyberlib.math.units.extensions.AngularVelocity
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.rpm
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
        get() = native.speed.rpm
        set(value) {
            log("Don't velocity control servos", LogMode.WARN)
            native.speed = value.rpm  // todo: mess with this conversion, speed is between -1 & 1
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
        get() = TODO("Not yet implemented")
        set(value) {}
    override var identifier: String
        get() = TODO("Not yet implemented")
        set(value) {}
    override var rawPercent: Double
        get() = native.speed
        set(value) {native.speed = value}

    override fun followTarget(kmc: KBasicMotorController) {
        TODO("Not yet implemented")
    }
}
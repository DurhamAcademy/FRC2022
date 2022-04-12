package frc.kyberlib.motorcontrol

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.Encoder
import frc.kyberlib.math.units.extensions.*

/**
 * Allows for encoded controls using a motor and a seperate encoder
 */
class CompoundMotor(val basic: KBasicMotorController, val encoder: Encoder) : KMotorController() {
    private val pid = PIDController(0.0, 0.0, 0.0, 0.02)
    override fun updateNativeProfile(maxVelocity: AngularVelocity, maxAcceleration: AngularVelocity) {
//        throw Exception("Compound motors have not builtin controller")
    }

    override fun updateNativeControl(p: Double, i: Double, d: Double) {
        pid.p = p
        pid.i = i
        pid.d = d
    }

    private var offset = 0.degrees
    override fun resetPosition(position: Angle) {
        offset = position
        encoder.reset()
    }

    override var rawPosition: Angle
        get() = encoder.distance.radians + offset
        set(value) {
            basic.voltage = arbFFVolts + pid.calculate(positionError.radians * toNative)
        }
    override var rawVelocity: AngularVelocity
        get() = encoder.rate.radiansPerSecond
        set(value) {
            basic.voltage = arbFFVolts + pid.calculate(velocityError.radiansPerSecond * toNative)
        }
    override var brakeMode: BrakeMode
        get() = basic.brakeMode
        set(value) {
            basic.brakeMode = value
        }
    override var identifier: String
        get() = "Enhanced ${basic.identifier}"
        set(value) {
            basic.identifier = value
        }
    override var rawPercent: Double
        get() = basic.percent
        set(value) {
            basic.percent = value
        }

    override fun followTarget(kmc: KBasicMotorController) {
        basic.follow(kmc)
    }

}
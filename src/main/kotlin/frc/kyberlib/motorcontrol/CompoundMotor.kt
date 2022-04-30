package frc.kyberlib.motorcontrol

import edu.wpi.first.wpilibj.Encoder
import frc.kyberlib.math.units.extensions.*

/**
 * Allows for encoded controls using a motor and a seperate encoder
 */
class CompoundMotor(val basic: KBasicMotorController, val encoder: Encoder) : KMotorController() {

    private var offset = 0.degrees
    override fun implementNativeControls(slot: Int) {
        throw IllegalCallerException("This doesn't have a integrated controller")
    }

    override var currentLimit: Int = 100

    override fun resetPosition(position: Angle) {
        offset = position
        encoder.reset()
    }

    override var rawPosition: Angle
        get() = encoder.distance.radians + offset
        set(value) {
            throw IllegalCallerException("This doesn't have a integrated controller")
        }
    override var rawVelocity: AngularVelocity
        get() = encoder.rate.radiansPerSecond
        set(value) {
            throw IllegalCallerException("This doesn't have a integrated controller")
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

    override var current: Double
        get() = if(motorConfigured) motorType!!.getCurrent(rawVelocity.value, voltage) else throw IllegalCallerException("This doesn't have a integrated controller")
        set(value) {
            // I = -1.0 / KvRadPerSecPerVolt / rOhms * speedRadiansPerSec + 1.0 / rOhms * voltageInputVolts;
            // I + speedRadiansPerSec/(KvRadPerSecPerVolt * rOhms) = voltage / rOmhs
            voltage = motorType!!.rOhms*(value + rawVelocity.value/(motorType!!.KvRadPerSecPerVolt * motorType!!.rOhms))
        }

    override fun followTarget(kmc: KBasicMotorController) {
        basic.follow(kmc)
    }

}
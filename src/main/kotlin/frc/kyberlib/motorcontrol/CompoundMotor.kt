package frc.kyberlib.motorcontrol

import edu.wpi.first.wpilibj.Encoder
import frc.kyberlib.math.units.extensions.*

/**
 * Allows for encoded controls using a motor and a seperate encoder
 */
class CompoundMotor(val basic: KBasicMotorController, val encoder: Encoder) : KMotorController() {

    private var offset = 0.degrees
    override fun implementNativeControls() {
        throw IllegalCallerException("This doesn't have a integrated controller")
    }

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

    override fun followTarget(kmc: KBasicMotorController) {
        basic.follow(kmc)
    }

}
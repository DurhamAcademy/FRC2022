package kyberlib.motorcontrol

import kyberlib.math.units.extensions.Angle
import kyberlib.math.units.extensions.AngularVelocity
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.rpm

/**
 * Raw simulated motor.
 */
class KSimulatedESC(name: String) : KMotorController() {  // potentially remove this and make abstra
    override var rawPosition: Angle = 0.degrees

    override var rawVelocity: AngularVelocity = 0.rpm
    override var currentLimit: Int = -1

    override fun configureEncoder(config: KEncoderConfig) = true

    override var identifier = name

    override var brakeMode: BrakeMode = false

    override var rawReversed: Boolean = false

    override var rawPercent: Double = 0.0

    override fun writePid(p: Double, i: Double, d: Double) {}

    override fun writeMultipler(mv: Double, mp: Double) {}

    override fun resetPosition(position: Angle) { this.position = position }

    override fun followTarget(kmc: KBasicMotorController) {
        kmc.followers.add(this)
        kmc.notifier.startPeriodic(0.005)
    }
}

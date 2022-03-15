package frc.kyberlib.motorcontrol

import frc.kyberlib.math.units.extensions.Angle
import frc.kyberlib.math.units.extensions.AngularVelocity
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.rpm

/**
 * Raw simulated motor.
 */
class KSimulatedESC(name: Any) : KMotorController() {  // potentially remove this and make abstract
    override var rawPosition: Angle = 0.degrees

    override var rawVelocity: AngularVelocity = 0.rpm

    override var identifier = name.toString()

    override var brakeMode: BrakeMode = false

    override var rawPercent: Double = 0.0

    override fun resetPosition(position: Angle) { this.position = position }

    override fun followTarget(kmc: KBasicMotorController) {
        if(!real) kmc.followers.add(this)
    }
}

package frc.kyberlib.motorcontrol

import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.*

/**
 * Raw simulated motor.
 */
class KSimulatedESC(name: Any) : KMotorController() {  // potentially remove this and make abstra
    override var rawPosition: Angle = 0.degrees

    override var rawVelocity: AngularVelocity = 0.rpm
    override var currentLimit: Int = -1

    override var identifier = name.toString()

    override var brakeMode: BrakeMode = false

    override var rawReversed: Boolean = false

    override var rawPercent: Double = 0.0

    override fun resetPosition(position: Angle) { this.position = position }

    override fun followTarget(kmc: KBasicMotorController) {
        if (Game.sim) {
            kmc.followers.add(this)
            kmc.notifier.startPeriodic(followPeriodic.seconds)
        }
    }
}

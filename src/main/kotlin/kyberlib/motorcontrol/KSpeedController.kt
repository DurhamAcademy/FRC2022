package kyberlib.motorcontrol

import edu.wpi.first.wpilibj.SpeedController

/**
 * Wraps a WPI SpeedController to use the KBasicMotorController API
 */
class KSpeedController(private val m_speedController: SpeedController) : KBasicMotorController() {
    private companion object{
        var id = 1
    }

    private val myId = id
    init { id += 1 }

    override var brakeMode: BrakeMode  // this doesn't work
        get() = false
        set(value) {}
    override var rawReversed: Boolean
        get() = m_speedController.inverted
        set(value) {m_speedController.inverted = value}
    override var identifier: String = "KSpeedController #$myId"
    override var rawPercent: Double
        get() = m_speedController.get()
        set(value) {m_speedController.set(value)}

    override fun followTarget(kmc: KBasicMotorController) {
        if (kmc.followers.size == 0) kmc.notifier.startPeriodic(0.005)
        kmc.followers.add(this)
    }
}

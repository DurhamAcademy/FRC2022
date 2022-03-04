package frc.kyberlib.motorcontrol

import edu.wpi.first.wpilibj.motorcontrol.MotorController

/**
 * Wraps a WPI SpeedController to use the KBasicMotorController API
 */
class KSpeedController(private val m_speedController: MotorController) : KBasicMotorController() {
    private companion object{
        var id = 1
    }

    private val myId = id
    init { id += 1 }

    override var brakeMode: BrakeMode = false  // this doesn't work
    override var identifier: String = "KSpeedController #$myId"
    override var rawPercent: Double
        get() = m_speedController.get()
        set(value) {m_speedController.set(value)}

    override fun followTarget(kmc: KBasicMotorController) {
        kmc.followers.add(this)
    }
}

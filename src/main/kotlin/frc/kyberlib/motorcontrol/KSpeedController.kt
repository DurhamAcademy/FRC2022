package frc.kyberlib.motorcontrol

import edu.wpi.first.wpilibj.motorcontrol.Jaguar
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj.motorcontrol.NidecBrushless
import edu.wpi.first.wpilibj.motorcontrol.PWMVenom

/**
 * Wraps a WPI SpeedController to use the KBasicMotorController API
 */
open class KSpeedController(private val m_speedController: MotorController, fake: Boolean = false) : KBasicMotorController(fake) {
    override var brakeMode: BrakeMode = false  // this doesn't work
    override var identifier: String = "KSpeedController"
    override var rawPercent: Double
        get() = m_speedController.get()
        set(value) {m_speedController.set(value)}

    override fun followTarget(kmc: KBasicMotorController) {
        kmc.followers.add(this)
    }
}

// couple other motor types we don't even own for kicks
class KJaguar(port: Int, fake: Boolean = false) : KSpeedController(Jaguar(port), fake)
class KVenom(port: Int, fake: Boolean = false) : KSpeedController(PWMVenom(port), fake)
class KNidec(port: Int, dio: Int, fake: Boolean = false) : KSpeedController(NidecBrushless(port, dio), fake)
package frc.kyberlib.motorcontrol.other

import edu.wpi.first.wpilibj.motorcontrol.Jaguar
import frc.kyberlib.motorcontrol.BrakeMode
import frc.kyberlib.motorcontrol.KBasicMotorController

class KJaguar(channel: Int, fake: Boolean = false) : KBasicMotorController(fake) {
    val t = Jaguar(channel)
    override var brakeMode: BrakeMode = false
        set(value) {throw IllegalCallerException("Jaguars can't set brake mode")}
    override var identifier: String = "Jaguar"
    override var rawPercent: Double
        get() = t.get()
        set(value) {t.set(value)}

    override fun followTarget(kmc: KBasicMotorController) {
        kmc.followers.add(this)
    }


}
package frc.kyberlib.pneumatics

import edu.wpi.first.networktables.NTSendable
import edu.wpi.first.networktables.NTSendableBuilder
import edu.wpi.first.wpilibj.*
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game

open class KSolenoid(fwd: Int, back: Int, private val fake: Boolean = false) : Debug, NTSendable {
    companion object {
        val allSolenoids = mutableListOf<KSolenoid>()
        val hub: PneumaticsBase = PneumaticsControlModule(1)
        val compressor = hub.makeCompressor().apply {
            enableDigital()
        }
    }

    init {
        allSolenoids.add(this)
    }
    override var identifier: String = "Pneumatic$fwd"
    private val solenoid: DoubleSolenoid = hub.makeDoubleSolenoid(fwd, back)
    var extended: Boolean = false
        get() = if (Game.real || fake) field else solenoid.get() == DoubleSolenoid.Value.kForward
        set(value) {if(Game.sim || fake) field = value else solenoid.set(if(value) DoubleSolenoid.Value.kForward else DoubleSolenoid.Value.kReverse)}

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "extended" to extended
        )
    }

    override fun initSendable(builder: NTSendableBuilder?) {
        builder!!.addBooleanProperty("extended", {extended}, {extended = it})
    }
}
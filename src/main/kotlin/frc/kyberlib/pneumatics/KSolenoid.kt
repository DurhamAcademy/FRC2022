package frc.kyberlib.pneumatics

import edu.wpi.first.networktables.NTSendable
import edu.wpi.first.networktables.NTSendableBuilder
import edu.wpi.first.wpilibj.*
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game

class KSolenoid(port: Int, private val fake: Boolean = false) : Debug, NTSendable {
    companion object {
        val allSolenoids = mutableListOf<KSolenoid>()
        val hub: PneumaticsBase = PneumaticsControlModule()
        val compressor = hub.makeCompressor().apply {
            enableDigital()
        }
    }

    init {
        allSolenoids.add(this)
    }
    override var identifier: String = "Pneumatic$port"
    private val solenoid = hub.makeSolenoid(port)
    var extended: Boolean = false
        get() = if (Game.real || fake) field else solenoid.get()
        set(value) {if(Game.sim || fake) field = value else solenoid.get()}

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "extended" to extended
        )
    }

    override fun initSendable(builder: NTSendableBuilder?) {
        builder!!.addBooleanProperty("extended", {extended}, {extended = it})
    }
}
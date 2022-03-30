package frc.kyberlib.pneumatics

import edu.wpi.first.networktables.NTSendable
import edu.wpi.first.networktables.NTSendableBuilder
import edu.wpi.first.wpilibj.*
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game

class KSolenoid(vararg val ports: Int, private val fake: Boolean = false) : Debug, NTSendable {
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
    override var identifier: String = "Pneumatic$ports"
    val solenoids = ports.map { hub.makeSolenoid(it) }
    var extended: Boolean
        get() = extension != 0
        set(value) {extension = if(value) 1 else 0}

    var extension: Int = 0
        set(value) {
            if(Game.real && !fake) {
                if(solenoids.size == 1) {
                    solenoids.first().set(value > 0)
                }
                else {
                    solenoids.forEachIndexed{index, solenoid ->
                        if(index <= value) solenoid.set(true)
                        else solenoid.set(false)
                    }
                }
            }
            field = value
        }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "extended" to extended
        )
    }

    override fun initSendable(builder: NTSendableBuilder?) {
        builder!!.addBooleanProperty("extended", {extended}, {extended = it})
    }
}
package frc.kyberlib.pneumatics

import edu.wpi.first.networktables.NTSendable
import edu.wpi.first.networktables.NTSendableBuilder
import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsBase
import edu.wpi.first.wpilibj.PneumaticsControlModule
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game

class KSolenoid(back: Int?, vararg fwd: Int, fake: Boolean = false) : Debug, NTSendable {
    companion object {
        val allSolenoids = mutableListOf<KSolenoid>()
        val hub: PneumaticsBase = PneumaticsControlModule(1)  // represents a PCM
        val compressor: Compressor = hub.makeCompressor().apply {  // represents a compressor
            enableDigital()
        }
    }
    private val followers = mutableListOf<KSolenoid>()
    private val real = !fake && Game.real

    init {
        allSolenoids.add(this)
    }

    override var identifier: String = "Pneumatic$fwd"

    private val backSolenoid = if(back != null) hub.makeSolenoid(back) else null
    private val fwdSoleniods = fwd.map { hub.makeSolenoid(it) }

    private var reverse: Boolean
        get() = backSolenoid?.get() ?: false
        set(value) { backSolenoid?.set(value) }

    /**
     * Whether the solenoid is extended or retracted
     */
    var extended: Boolean
        get() = extension > 0
        set(value) {
            extension = if(value) 1 else 0
        }

    /**
     * How many pistons out the solenoid is.
     * 0 = back, 1 = out, 2+ = further out
     */
    var extension: Int = 0  // attempt to make a variable extension length pneumatic system. Wasn't working but didn't test much
        set(value) {
            if (real) {
                reverse = value <= 0
                fwdSoleniods.forEachIndexed { index, solenoid -> solenoid.set(index < value) }
            }
            field = value
            followers.forEach { it.extension = value }
        }

    fun follow(other: KSolenoid) {other.followers.add(this)}

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "extended" to extended
        )
    }

    override fun initSendable(builder: NTSendableBuilder?) {
        builder!!.addBooleanProperty("extended", { extended }, { extended = it })
    }
}
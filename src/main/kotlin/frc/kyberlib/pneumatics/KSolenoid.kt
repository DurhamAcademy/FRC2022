package frc.kyberlib.pneumatics

import edu.wpi.first.networktables.NTSendable
import edu.wpi.first.networktables.NTSendableBuilder
import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsBase
import edu.wpi.first.wpilibj.PneumaticsControlModule
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game

class KSolenoid(back: Int, fwd: Int, fake: Boolean = false) : Debug, NTSendable {
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
    private val double = hub.makeDoubleSolenoid(fwd, back)

    //    val solenoids = ports.map { hub.makeSolenoid(it) }
    /**
     * Whether the solenoid is extended or retracted
     */
    var extended: Boolean = false
        get() = if (real) double.get() == DoubleSolenoid.Value.kForward else field//extension != 0
        set(value) {
            if (real)
                double.set(if (value) DoubleSolenoid.Value.kForward else DoubleSolenoid.Value.kReverse)
            else field = value
            followers.forEach { it.extended = value }
        }//extension = if(value) 1 else 0}

    /**
     * How many pistons out the solenoid is.
     * 0 = back, 1 = out, 2+ = further out
     */
    var extension: Int = 0  // attempt to make a variable extension length pneumatic system. Wasn't working but didn't test much
        set(value) {
            if (real) {
//                if(solenoids.size == 1) {
//                    solenoids.first().set(value > 0)
//                }
//                else {
//                    solenoids.forEachIndexed{index, solenoid ->
//                        if(index <= value) solenoid.set(true)
//                        else solenoid.set(false)
//                    }
//                }
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
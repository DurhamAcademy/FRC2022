package frc.kyberlib.input

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.kyberlib.command.Debug

/**
 * Wrapper for a DoubleSuppler (that retrieves raw axis value) and applies some hyper-parameters to increase usability.
 *
 * Explination: https://oscarliang.com/rc-roll-pitch-yaw-rate-cleanflight/
 *
 * Graph: https://www.desmos.com/calculator/mxwvyq14yp
 */
class KAxis(val raw: () -> Double) : Debug {
    companion object {
        val all = mutableListOf<KAxis>()
    }
    init {
        all.add(this)
    }
    var simVal = 0.0

    var maxVal = 1.0  // the adjusted max val that the axis will output (rate in explination)
    var expo = 0.0  // increases
    var deadband = 0.01  // how much of the controller should default to 0
    // superrate is deprecated because dumb


    /**
     * Applies fancy math to make joystick non-linear
     */
    private fun modify(unmodified: Double): Double {
        // apply deadband
        val command = when {
            unmodified > deadband -> (1 / (1 - deadband)) * unmodified - (deadband / (1 - deadband))
            unmodified < -deadband -> (1 / (1 - deadband)) * unmodified + (deadband / (1 - deadband))
            else -> 0.0
        }

        // apply expo
        val exponetialized = (1 + 0.01 * expo * (command * command - 1.0)) * command

        // adjust to maxVal
        return exponetialized * maxVal
    }

    /**
     * Fancy non-linear value of the axis
     */
    val value: Double
        get() = if(KController.sim) simVal else modify(raw.invoke())

    fun activateAt(value: Double = 0.5) = Trigger { this.raw() > value }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "raw" to raw.invoke(),
            "value" to value
        )
    }
}

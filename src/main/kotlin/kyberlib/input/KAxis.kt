package kyberlib.input

import kyberlib.command.Debug

/**
 * Wrapper for a DoubleSuppler (that retrieves raw axis value) and applies some hyper-parameters to increase usability.
 *
 * Explination: https://oscarliang.com/rc-roll-pitch-yaw-rate-cleanflight/
 *
 * Graph: https://www.desmos.com/calculator/mxwvyq14yp
 */
class KAxis(val raw: () -> Double) : Debug {
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
        get() = modify(raw.invoke())

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "raw" to raw.invoke(),
            "value" to value
        )
    }
}

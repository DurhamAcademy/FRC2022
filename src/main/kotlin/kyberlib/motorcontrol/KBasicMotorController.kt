package kyberlib.motorcontrol

import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import kyberlib.command.Debug
import kyberlib.math.invertIf

/**
 * A basic motor controller. No closed-loop control
 */
abstract class KBasicMotorController : Sendable, Debug {
    protected var controlMode = ControlMode.NULL
    // ------ configs ----- //
    /**
     * Controls how the motor will stop when set to 0. If true the motor will brake instead of coast.
     */
    abstract var brakeMode: BrakeMode

    /**
     * Determines if the motor should run in the opposite direction
     */
    var reversed: Boolean = false
        get() = if (real) rawReversed else field
        set(value) {if (real) rawReversed else field = value}

    abstract var rawReversed: Boolean

    /**
     * The prefix used by this motor for logging of errors and debug information.
     */
    abstract override var identifier: String

    /**
     * Whether the motor is connected to a real Robot
     */
    protected val real: Boolean
        get() = RobotBase.isReal()

    // ------ low-level write methods ----- //
    /**
     * Sets the voltage without changing the control mode
     */
    protected fun safeSetVoltage(voltage: Double) {
        val prevMode = controlMode
        this.voltage = voltage
        controlMode = prevMode
    }
    /**
     * What percent output is currently being applied?
     */
    var percent: Double = 0.0
        get() = if (real) rawPercent.invertIf { reversed } else field
        set(value) {
            val adjusted = value
            controlMode = ControlMode.VOLTAGE
            if (real) rawPercent = adjusted else field = adjusted
        }

    /**
     * Native level get and set of motor percent.
     * Recommend using Percent because it is safer
     */
    protected abstract var rawPercent: Double

    /**
     * Sets controller voltage directly
     */
    var voltage: Double
        get() = percent * vbus
        set(value) {
            value.coerceIn(0.0 , vbus)
            percent = (value / vbus)
        }

    /**
     * The voltage available to the motor
     */
    private val vbus = if (real) RobotController.getBatteryVoltage() else 12.0

    /**
     * The notifier to update the motor continuously
     */
    internal val notifier = Notifier { update() }

    /**
     * True if this motor is following another.
     */
    var isFollower = false
        protected set

    operator fun plusAssign(kmc: KBasicMotorController) {
        kmc.follow(this)
    }

    internal val followers = arrayListOf<KBasicMotorController>()

    /**
     * Follow a motor
     */
    fun follow(kmc: KBasicMotorController) {
        isFollower = true
        followTarget(kmc)
    }

    /**
     * Native level follow
     */
    protected abstract fun followTarget(kmc: KBasicMotorController)

    /**
     * Internal update function
     */
    open fun update() {
        updateFollowers()
    }

    /**
     * Update all followers to match voltage
     */
    private fun updateFollowers() {
        for (follower in followers) {
             follower.percent = percent.invertIf { follower.reversed }
             follower.update()
        }
    }

    /**
     * Converts motor in sendable graphics widget
     */
    override fun initSendable(builder: SendableBuilder) {
        builder.setSmartDashboardType("Encoder")
        builder.addDoubleProperty("Voltage", this::voltage) { this.voltage = it }
        builder.addBooleanProperty("brake mode", this::brakeMode, this::brakeMode::set)
        builder.addBooleanProperty("reversed", this::reversed, this::reversed::set)
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "Voltage" to voltage,
            "brake mode" to brakeMode,
            "reversed" to reversed
        )
    }

    override fun toString(): String {
        return "Motor($identifier)"
    }
}

package frc.kyberlib.motorcontrol

import edu.wpi.first.wpilibj.*
import edu.wpi.first.networktables.NTSendable
import edu.wpi.first.networktables.NTSendableBuilder
import frc.kyberlib.command.*
import frc.kyberlib.math.invertIf
import frc.kyberlib.math.units.extensions.Time
import frc.kyberlib.math.units.extensions.seconds

/**
 * A basic motor controller. No closed-loop control
 */
abstract class KBasicMotorController : NTSendable, Debug {
    companion object {
        val allMotors = mutableListOf<KBasicMotorController>()
    }

    init { addReferences() }
    private fun addReferences() {
        allMotors.add(this)
    }
    protected val followPeriodic = 0.005.seconds
    var controlMode = ControlMode.NULL
    // ------ configs ----- //
    /**
     * Controls how the motor will stop when set to 0. If true the motor will brake instead of coast.
     */
    abstract var brakeMode: BrakeMode

    /**
     * Determines if the motor should run in the opposite direction
     */
    var reversed: Boolean = false

    abstract var rawReversed: Boolean

    /**
     * The prefix used by this motor for logging of errors and debug information.
     */
    abstract override var identifier: String

    /**
     * Whether the motor is connected to a real Robot
     */
    protected val real: Boolean = Game.real

    // ------ low-level write methods ----- //
    /**
     * Sets the voltage without changing the control mode
     */
    protected fun safeSetVoltage(v: Double) {
        val prevMode = controlMode
        voltage = v
        controlMode = prevMode
    }
    /**
     * What percent output is currently being applied?
     */
    var percent: Double
        get() = rawPercent
        set(value) {
            val adjusted = value.invertIf { reversed }
            controlMode = ControlMode.VOLTAGE
            if (real) rawPercent = adjusted
        }

    /**
     * Native level get and set of motor percent.
     * Recommend using Percent because it is safer
     */
    protected abstract var rawPercent: Double

    var maxVoltage = 13.0
    /**
     * Sets controller voltage directly
     */
    var voltage: Double
        get() = percent * vbus
        set(value) {
            val norm = value.coerceIn(-vbus , vbus).coerceIn(-maxVoltage, maxVoltage)
            percent = (norm / vbus)
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
     * Halts the motor
     */
    open fun stop() {
        safeSetVoltage(0.0)
    }

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
//             follower.update()
        }
    }

    open fun checkError(): Boolean = false

    /**
     * Converts motor in NTSendable graphics widget
     */
    override fun initSendable(builder: NTSendableBuilder) {
        builder.setSmartDashboardType("Encoder")
        builder.addStringProperty("Control Type", {controlMode.name}, {})
        builder.addDoubleProperty("Voltage", this::voltage) { if(it != voltage) this.voltage = it }
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

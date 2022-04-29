package frc.kyberlib.motorcontrol

import edu.wpi.first.networktables.NTSendable
import edu.wpi.first.networktables.NTSendableBuilder
import edu.wpi.first.wpilibj.RobotController
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.math.invertIf
import frc.kyberlib.math.units.extensions.seconds

typealias Voltage = Double
/**
 * A basic motor controller. No closed-loop control
 */
abstract class KBasicMotorController(fake: Boolean = false) : NTSendable, Debug {
    protected val followPeriodic = 0.1.seconds
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

    /**
     * The prefix used by this motor for logging of errors and debug information.
     */
    abstract override var identifier: String

    /**
     * Whether the motor is connected to a real Robot
     */
    val real: Boolean = Game.real && !fake

    // ------ low-level write methods ----- //
    /**
     * Sets the voltage without changing the control mode
     */
    protected fun safeSetVoltage(v: Voltage) {
        val prevMode = controlMode
        voltage = v
        controlMode = prevMode
    }

    /**
     * What percent output is currently being applied?
     */
    var percent: Double = 0.0
        get() = if (Game.real) rawPercent else field
        set(value) {
            val adjusted = value.invertIf { reversed }
            controlMode = ControlMode.VOLTAGE
            for (follower in followers) follower.percent = adjusted
            if (real) rawPercent = adjusted else field = value
        }

    /**
     * Native level get and set of motor percent.
     * Recommend using Percent because it is safer
     */
    protected abstract var rawPercent: Double

    /**
     * Sets controller voltage directly
     */
    var voltage: Voltage
        inline get() = percent * vbus
        inline set(value) {
            val norm = value.coerceIn(-maxVoltage, maxVoltage)
            percent = (norm / vbus)
        }

    /**
     * The voltage available to the motor
     */
    val vbus
        inline get() = if (real) RobotController.getBatteryVoltage() else 12.0

    /**
     * Max voltage able to be applied to the motor
     */
    var maxVoltage: Voltage = vbus
        set(value) {
            field = value.coerceIn(0.0, vbus)
        }

    /**
     * True if this motor is following another.
     */
    protected var isFollower = false

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

    open fun checkError(): Boolean = false

    /**
     * Converts motor in NTSendable graphics widget
     */
    override fun initSendable(builder: NTSendableBuilder) {
        builder.setSmartDashboardType("Encoder")
        builder.setActuator(true)
        builder.setSafeState(this::stop)
        builder.addStringProperty("Control Type", { controlMode.name }, null)
        builder.addDoubleProperty("Voltage", this::voltage) { if (it != voltage) this.voltage = it }
    }

    override fun toString(): String {
        return "Motor($identifier)"
    }
}

open class BasicMotorReader(private val motor: KBasicMotorController) {
    val percent get() = motor.percent
    val voltage get() = motor.voltage
    val real get() = motor.real
    val brakeMode get() = motor.brakeMode
    val reversed get() = motor.reversed
}

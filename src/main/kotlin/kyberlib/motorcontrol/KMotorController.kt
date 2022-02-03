package kyberlib.motorcontrol

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.controller.ArmFeedforward
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import kyberlib.math.filters.Differentiator
import kyberlib.math.units.extensions.*

typealias GearRatio = Double
typealias BrakeMode = Boolean

/**
 * Types of encoders that may be used
 */
enum class EncoderType {
    NONE, NEO_HALL, QUADRATURE
}

/**
 * Defines types of motors that can be used
 */
enum class MotorType {
    BRUSHLESS, BRUSHED
}

/**
 * The type update goal the motor is going for
 */
enum class ControlMode {
    VELOCITY, POSITION, VOLTAGE, NULL
}

/**
 * Stores data about an encoder. [reversed] means the encoder reading goes + when the motor is applied - voltage.
 */
data class KEncoderConfig(val cpr: Int, val type: EncoderType, val reversed: Boolean = false)
/**
 * A more advanced motor control with feedback control.
 */
abstract class KMotorController : KBasicMotorController() {
    // find a way to simulate brake mode
    // ----- configs ----- //
    /**
     * The multiplier to attach to the raw velocity. *This is not the recommended way to do this.* Try using gearRatio instead
     */
    var velocityConversionFactor = 1.0
        set(value) {
            field = value
            writeMultipler(velocityConversionFactor, positionConversionFactor)
        }

    /**
     * The multiplier to attach to the raw position. *This is not the recommended way to do this.* Try using gearRatio instead
     */
    var positionConversionFactor = 1.0
        set(value) {
            field = value
            writeMultipler(velocityConversionFactor, positionConversionFactor)
        }

    /**
     * Defines the relationship between rotation and linear motion for the motor.
     */
    var radius: Length? = null

    /**
     * Adds post-encoder gearing to allow for post-geared speeds to be set.
     */
    var gearRatio: GearRatio = 1.0
        set(value) {
            field = value
//            writeMultipler(gearRatio.rpm.rotationsPerSecond, gearRatio)
        }

    /**
     * Settings relevant to the motor controller's encoder.
     */
    var encoderConfig: KEncoderConfig = KEncoderConfig(0, EncoderType.NONE)
        set(value) {
            if (configureEncoder(value)) field = value
            else System.err.println("Invalid encoder configuration")
        }

    // ----- advanced tuning ----- //
    /**
     * Proportional gain of the customControl controller.
     */
    var kP: Double
        get() = PID.p
        set(value) {
            PID.p = value
            writePid(kP, kI, kD)
        }

    /**
     * Integral gain of the customControl controller.
     */
    var kI: Double
        get() = PID.i
        set(value) {
            PID.i = value
            writePid(kP, kI, kD)
        }

    /**
     * Derivative gain of the customControl controller.
     */
    var kD: Double
        get() = PID.d
        set(value) {
            PID.d = value
            writePid(kP, kI, kD)
        }

    /**
     * The max angular velocity the motor can have
     */
    var maxVelocity: AngularVelocity
        get() = constraints.maxVelocity.radiansPerSecond
        set(value) { constraints.maxVelocity = value.radiansPerSecond }
    /**
     * The max angular acceleation the motor can have
     */
    var maxAcceleration: AngularVelocity
        get() = constraints.maxAcceleration.radiansPerSecond
        set(value) { constraints.maxAcceleration = value.radiansPerSecond }
    /**
     * The max linear velocity the motor can have
     */
    var maxLinearVelocity: LinearVelocity
        get() = rotationToLinear(maxVelocity)
        set(value) { maxVelocity = linearToRotation(value) }
    /**
     * The max linear acceleration the motor can have
     */
    var maxLinearAcceleration: LinearVelocity
        get() = rotationToLinear(maxAcceleration)
        set(value) { maxAcceleration = linearToRotation(value) }

    private val constraints = TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY)
    var PID = PIDController(0.0, 0.0, 0.0)  // what does the profile do?

    /**
     * Builtin control that will combine feedforward with the PID.
     * Useful in both position and velocity control
     */
    fun addFeedforward(feedforward: SimpleMotorFeedforward) {
        customControl = {
            when (controlMode) {
                ControlMode.VELOCITY -> {
                    val ff = feedforward.calculate(linearVelocitySetpoint.metersPerSecond, linearAcceleration.metersPerSecond)
                    val pid = PID.calculate(linearVelocityError.metersPerSecond)
                    ff + pid
                }
                ControlMode.POSITION -> {
                    val pid = PID.calculate(it.positionError.radians)
                    pid
                }
                ControlMode.VOLTAGE -> voltage
                else -> 0.0
            }
        }
    }

    /**
     * Used to create builtin functions for angular position sheet.
     * @exception WhyFuckingLinear you should not have linear controlled arm.
     */
    fun addFeedforward(feedforward: ArmFeedforward) {
        assert(!linearConfigured) {"you shouldn't be using armFF for linear motors"}
        customControl = {
            when (controlMode) {
                ControlMode.POSITION -> {
                    val ff = feedforward.calculate(position.radians, velocity.radiansPerSecond, acceleration.radiansPerSecond)
                    val pid = PID.calculate(positionError.radians)
                    ff + pid
                }
                ControlMode.VELOCITY -> {
                    val ff = feedforward.calculate(position.radians, velocity.radiansPerSecond, acceleration.radiansPerSecond)
                    val pid = PID.calculate(velocityError.radiansPerSecond)
                    ff + pid
                }
                ControlMode.VOLTAGE -> voltage
                else -> 0.0
            }

        }
    }

    /**
     * Follows a trapezoidal motion profile and then reverts when done
     */
    fun followProfile(profile: TrapezoidProfile) {
        val timer = Timer()
        val prevControl = customControl
        timer.start()
        customControl = {
            val state = profile.calculate(timer.get())
            position = state.position.radians
            if (profile.isFinished(timer.get())) {
                timer.stop()
                customControl = prevControl
            }
            if (prevControl != null) prevControl(it) else 0.0
        }
    }

    /**
     * The control function of the robot.
     * Uses the motor state to determine what voltage should be applied.
     * Can be set to null to use in-built motor control system
     */
    var customControl: ((it: KMotorController) -> Double)? = {
        when (controlMode) {
            ControlMode.POSITION -> PID.calculate(positionError.radians)
            ControlMode.VELOCITY -> PID.calculate(velocityError.radiansPerSecond)
            ControlMode.VOLTAGE -> it.voltage
            else -> 0.0
        }
    }

    // ----- main getter/setter methods ----- //
    /**
     * Angle that the motor is at / should be at
     */
    var position: Angle
        get() {
            if (!real) return simPosition
            assert(encoderConfigured)
            return (rawPosition.value * gearRatio).radians
        }
        set(value) {
            controlMode = ControlMode.POSITION
            positionSetpoint = value
        }

    /**
     * Linear Position that the
     */
    var linearPosition: Length
        get() = rotationToLinear(position)
        set(value) { position = linearToRotation(value) }

    var velocity: AngularVelocity
        get() {
            assert(encoderConfigured) {"configure your motor before using"}
            val vel = if (real) rawVelocity * gearRatio else simVelocity
//            acceleration = accelerationCalculator.calculate(vel.radiansPerSecond).radiansPerSecond  // todo: acc doesnt work in sims - find workarounds
            return vel
        }
        set(value) {
            controlMode = ControlMode.VELOCITY
            velocitySetpoint = value
        }

    var linearVelocity: LinearVelocity
        get() = rotationToLinear(velocity)
        set(value) { velocity = linearToRotation(value) }

    val positionError
        get() = position - positionSetpoint
    val linearPositionError
        get() = linearPosition - linearPositionSetpoint
    val velocityError
        get() = velocity - velocitySetpoint
    val linearVelocityError
        get() = linearVelocity - linearVelocitySetpoint

    private var accelerationCalculator = Differentiator()
    var acceleration = 0.rpm
    val linearAcceleration
        get() = rotationToLinear(acceleration)

    // ----- this is where you put the advanced controls ---- //
    /**
     * Sets the angle to which the motor should go
     */
    var positionSetpoint: Angle = 0.rotations
        private set(value) {
            field = value
            if(!closedLoopConfigured && real) rawPosition = value
            else updateVoltage()
        }

    /**
     * Sets the velocity to which the motor should go
     */
    var velocitySetpoint: AngularVelocity = 0.rpm
        private set(value) {
            field = value
            if (!closedLoopConfigured && real) rawVelocity = value
            else updateVoltage()
        }

    /**
     * Sets the linear position to which the motor should go
     */
    val linearPositionSetpoint: Length
        get() = rotationToLinear(positionSetpoint)

    /**
     * Sets the linear velocity to which the motor should go
     */
    val linearVelocitySetpoint: LinearVelocity
        get() = rotationToLinear(velocitySetpoint)

    // ----- util functions -----//
    /**
     * Converts linear units to rotational units
     * @exception LinearUnconfigured: you must set wheel radius before using
     */
    private fun linearToRotation(len: Length): Angle {
        if (!linearConfigured) throw LinearUnconfigured
        return len.toAngle(radius!!)
    }
    private fun linearToRotation(vel: LinearVelocity): AngularVelocity {
        if (!linearConfigured) throw LinearUnconfigured
        return vel.toAngularVelocity(radius!!)
    }

    /**
     * Converts rotational units to linear
     * @exception LinearUnconfigured you must set wheel radius before using
     */
    private fun rotationToLinear(ang: Angle): Length {
        if (!linearConfigured) throw LinearUnconfigured
        return ang.toCircumference(radius!!)
    }
    private fun rotationToLinear(vel: AngularVelocity): LinearVelocity {
        if (!linearConfigured) throw LinearUnconfigured
        return vel.toTangentialVelocity(radius!!)
    }

    /**
     * Updates the voltage after changing position / velocity setpoint
     */
    private fun updateVoltage() {
        if (!isFollower && customControl != null)
            safeSetVoltage(customControl!!(this))
    }

    // ----- meta information ----- //
    /**
     * Does the motor controller have a rotational to linear motion conversion defined? (i.e. wheel radius)
     * Allows for linear units to be used.
     */
    private val linearConfigured
        get() = radius != null

    /**
     * Does the motor controller have an encoder configured?
     * Allows for closed-loop control methods to be used
     */
    protected val encoderConfigured
        get() = (encoderConfig.type != EncoderType.NONE && encoderConfig.cpr > 0)

    /**
     * Does the motor have closed-loop gains set?
     * Allows for closed-loop control methods to be used
     */
    private val closedLoopConfigured
        get() = encoderConfigured && customControl != null

    // ----- low level getters and setters (customized to each encoder type) ----- //
    /**
     * Resets motor to a certain positions
     * Does *not* move to angle, just changes the variable
     */
    abstract fun resetPosition(position: Angle = 0.rotations)

    /**
     * Resets where the encoder thinks it is.
     * Does *not* move motor to that spot, just internal variable.
     */
    fun resetPosition(position: Length) {
        resetPosition(linearToRotation(position))
    }

    /**
     * The native motors get and set methods for position.
     * Recommend using Position instead because it is safer.
     */
    abstract var rawPosition: Angle
        protected set

    /**
     * Native motor get and set for angular velocity.
     * Recommend using Velocity instead because it is safer
     */
    abstract var rawVelocity: AngularVelocity
        protected set

    /**
     * Max current draw for the motors
     */
    abstract var currentLimit: Int

    /**
     * Write pid values to native motor
     */
    protected abstract fun writePid(p: Double, i: Double, d: Double)

    /**
     * Set the conversion multipliers of to native motor
     */
    protected abstract fun writeMultipler(mv: Double, mp: Double)

    /**
     * Configures the respective ESC encoder settings when a new encoder configuration is set
     */
    protected abstract fun configureEncoder(config: KEncoderConfig): Boolean

    var simVelocity: AngularVelocity = 0.rpm
        set(value) {
            assert(!real) {"This value should only be set from a simulation"}
            field = value
        }
    var simLinearVelocity: LinearVelocity
        get() = rotationToLinear(simVelocity)
        set(value) { simVelocity = linearToRotation(value) }
    var simPosition: Angle = 0.degrees
        set(value) {
            assert(!real) {"This value should only be set from a simulation"}
            field = value
        }
    var simLinearPosition: Length
        get() = rotationToLinear(simPosition)
        set(value) { simPosition = linearToRotation(value) }

    override fun initSendable(builder: SendableBuilder) {
        super.initSendable(builder)
        builder.addDoubleProperty("Angular Position (rad)", {linearPosition.meters}, { linearPosition = it.meters })
        builder.addDoubleProperty("Angular Velocity (rad per s)", {linearVelocity.metersPerSecond}, { linearVelocity = it.metersPerSecond })
        if (linearConfigured) {
            builder.addDoubleProperty("Linear Position (m)", {linearPosition.meters}, { linearPosition = it.meters })
            builder.addDoubleProperty("Linear Velocity (m per s)", {linearVelocity.metersPerSecond}, { linearVelocity = it.metersPerSecond })
        }
    }
    override fun debugValues(): Map<String, Any?> {
        val map = super.debugValues().toMutableMap()
        map.putAll(mapOf(
            "Angular Position (rad)" to position.radians,
            "Angular Velocity (rad per s)" to velocity.radiansPerSecond
//            "Angular Acceleration (rad per s per s)" to acceleration.radiansPerSecond  // temporary (here for testing)
        ))
        if (linearConfigured)
            map.putAll(mapOf(
                "Linear Position (m)" to linearPosition.meters,
                "Linear Velocity (m per s)" to linearVelocity.metersPerSecond
            ))
//        map["PID"] = PID
        return map.toMap()
    }

    companion object LinearUnconfigured : Exception("You must set the wheel radius before using linear values")
}
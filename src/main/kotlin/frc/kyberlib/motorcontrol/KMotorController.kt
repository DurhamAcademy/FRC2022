package frc.kyberlib.motorcontrol

import edu.wpi.first.math.Num
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.*
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.networktables.NTSendableBuilder
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.simulation.LinearSystemSim
import frc.kyberlib.command.Game
import frc.kyberlib.command.LogMode
import frc.kyberlib.math.filters.Differentiator
import frc.kyberlib.math.invertIf
import frc.kyberlib.math.sign
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.simulation.Simulatable
import frc.kyberlib.simulation.Simulation
import java.util.function.Supplier
import kotlin.math.absoluteValue

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
enum class BrushType {
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
abstract class KMotorController : KBasicMotorController(), Simulatable {
    open var motorType: DCMotor? = null
    // ----- configs ----- //
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
        set(value) { PID.p = value }

    /**
     * Integral gain of the customControl controller.
     */
    var kI: Double
        get() = PID.i
        set(value) { PID.i = value }

    /**
     * Derivative gain of the customControl controller.
     */
    var kD: Double
        get() = PID.d
        set(value) { PID.d = value }

    /**
     * The max angular velocity the motor can have
     */
    var maxVelocity: AngularVelocity
        get() = constraints.maxVelocity.radiansPerSecond
        set(value) { constraints = TrapezoidProfile.Constraints(value.radiansPerSecond, constraints.maxVelocity) }
    /**
     * The max angular acceleation the motor can have
     */
    var maxAcceleration: AngularVelocity
        get() = constraints.maxAcceleration.radiansPerSecond
        set(value) { constraints = TrapezoidProfile.Constraints(constraints.maxVelocity, value.radiansPerSecond) }

    var maxPosition: Angle? = null
    var maxLinearPosition: Length?
        get() = maxPosition?.let { rotationToLinear(it) }
        set(value) {maxPosition = if (value == null) value else linearToRotation(value)}
    var minPosition: Angle? = null
    var minLinearPosition: Length?
        get() = minPosition?.let { rotationToLinear(it) }
        set(value) {minPosition = if (value == null) value else linearToRotation(value)}
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

    private var constraints = TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY)
        set(value) {
            field = value
            PID.setConstraints(value)
        }
    var PID = ProfiledPIDController(0.0, 0.0, 0.0, constraints)  // what does the profile do?

    /**
     * Builtin control that will combine feedforward with the PID.
     * Useful in both position and velocity control
     */
    fun addFeedforward(feedforward: SimpleMotorFeedforward) {
        customControl = {
            when (controlMode) {
                ControlMode.VELOCITY -> {
//                    val ff = feedforward.calculate(velocity.radiansPerSecond, -velocityError.radiansPerSecond / KRobot.period / feedforward.ka / 1.0)
                    val ff = feedforward.calculate(velocitySetpoint.radiansPerSecond)  // todo: check if better
                    val pid = PID.calculate(velocityError.radiansPerSecond)
                    ff + pid
                }
                ControlMode.POSITION -> {
                    val pid = PID.calculate(it.positionError.radians)
                    pid
                }
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
                    val ff = feedforward.calculate(position.radians, PID.calculate(positionError.radians))
                    ff
                }
                ControlMode.VELOCITY -> {
                    val ff = feedforward.calculate(position.radians, velocity.radiansPerSecond)
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
    fun followProfile(profile: TrapezoidProfile) {  // todo: follow this better
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
    /**
     * Locks recursive calls to customControl from inside of customControl
     */
    private var customControlLock = false

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

    // todo: documentation
    var linearPosition: Length
        get() = rotationToLinear(position)
        set(value) { position = linearToRotation(value) }

    var velocity: AngularVelocity
        get() {
            assert(encoderConfigured) {"configure your motor before using"}
            val vel = if (real) rawVelocity * gearRatio else simVelocity
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

    // todo: make optional
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
            var clamped = value
            if (minPosition != null) clamped = value.value.coerceAtLeast(minPosition!!.value).radians
            if (maxPosition != null) clamped = value.value.coerceAtMost(maxPosition!!.value).radians
            field = clamped
            if(!closedLoopConfigured && real) rawPosition = clamped
            else if (!customControlLock) updateVoltage()
        }

    /**
     * Sets the velocity to which the motor should go
     */
    var velocitySetpoint: AngularVelocity = 0.rpm
        private set(value) {
            field = value
            if (!closedLoopConfigured && real) rawVelocity = value
            else if (!customControlLock) updateVoltage()
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
    fun updateVoltage() {
        if (!isFollower && customControl != null && controlMode != ControlMode.VOLTAGE) {
            customControlLock = true  // todo: mitigate customControl crashes
            safeSetVoltage(customControl!!(this))
            customControlLock = false
        }
    }

    // ----- meta information ----- //
    /**
     * Does the motor controller have a rotational to linear motion conversion defined? (i.e. wheel radius)
     * Allows for linear units to be used.
     */
    private val linearConfigured
        get() = radius != null

    private val motorConfigured
        get() = motorType != null

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
     * Configures the respective ESC encoder settings when a new encoder configuration is set
     */
    protected abstract fun configureEncoder(config: KEncoderConfig): Boolean

    var simVelocity: AngularVelocity = 0.rpm
        set(value) {
            assert(!real) {"This value should only be set from a simulation"}
            field = value
        }
    var simPosition: Angle = 0.degrees
        set(value) {
            assert(!real) {"This value should only be set from a simulation"}
            field = value
        }
    var simLinearVelocity
        get() = rotationToLinear(simVelocity)
        set(value) {simVelocity = linearToRotation(value)}
    var simLinearPosition
        get() = rotationToLinear(simPosition)
        set(value) {simPosition = linearToRotation(value)}


    override fun initSendable(builder: NTSendableBuilder) {
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
            "Angular Position" to position,
            "Angular Velocity" to velocity,
            // "Angular Acceleration (rad per s per s)" to acceleration.radiansPerSecond  // temporary (here for testing)
        ))
        if (linearConfigured)
            map.putAll(mapOf(
                "Linear Position" to linearPosition,
                "Linear Velocity" to linearVelocity,
                // "Linear Acceleration (m per s per s)" to linearAcceleration.metersPerSecond
            ))
        if (controlMode == ControlMode.POSITION) {
            if (linearConfigured)
                map.putAll(mapOf(
                    "setpoint" to linearPositionSetpoint,
                    "error" to linearPositionError
                ))
            else
                map.putAll(mapOf(
                        "setpoint" to positionSetpoint,
                        "error" to positionError
                    ))
        }
        else {
            if (linearConfigured)
                map.putAll(mapOf(
                    "setpoint" to linearVelocitySetpoint.metersPerSecond,
                    "error" to linearVelocityError.metersPerSecond
                ))
            else
                map.putAll(mapOf(
                        "setpoint" to velocitySetpoint.radiansPerSecond,
                        "error" to velocityError.radiansPerSecond
                    ))
        }
//        map["PID"] = PID
        return map.toMap()
    }

    fun setupSim(feedforward: SimpleMotorFeedforward) {
        simKS = {feedforward.ks}
        simKV = feedforward.kv
        simKA = feedforward.ka
        Simulation.instance.include(this)
    }
    fun setupSim(feedforward: ArmFeedforward) {
        simKS = { feedforward.ks + feedforward.kcos * position.cos }
        simKV = feedforward.kv
        simKA = feedforward.ka
        if(!brakeMode) log("Use brakeMode", logMode = LogMode.WARN)
        Simulation.instance.include(this)
    }
    fun setupSim(feedforward: ElevatorFeedforward) {
        simKS = { feedforward.ks * voltage.sign + feedforward.kg }
        simKV = feedforward.kv
        simKA = feedforward.ka
        if(!brakeMode) log("Use brakeMode", logMode = LogMode.WARN)
        Simulation.instance.include(this)
    }
    private lateinit var simKS: () -> Double
    private var simKV = 0.0
    private var simKA = 0.0
    override fun simUpdate(dt: Time) {
        if(!this::simKS.isInitialized) return
        if(voltage.absoluteValue < simKS()) {
            simVelocity = 0.radiansPerSecond
            // kinda assuming brakeMode
        }
        else {
            val applicableVolt = (voltage.absoluteValue - simKS()).invertIf { voltage < 0.0 }
            val vel = velocity.radiansPerSecond
            val accVolt = applicableVolt - simKV * vel
            val acceleration = accVolt / simKA
            if (brakeMode) {
                val velMaintananceVolt = simKS() + simKV * velocity.radiansPerSecond.absoluteValue
                if (velMaintananceVolt.sign != velocity.radiansPerSecond.sign && vel != 0.0)
                    simVelocity = 0.radiansPerSecond
                else if (velMaintananceVolt < voltage.absoluteValue)
                    simVelocity = (applicableVolt / simKV).radiansPerSecond
                else
                    simVelocity += acceleration.radiansPerSecond * dt.seconds
            }
            else {
                simVelocity += acceleration.radiansPerSecond * dt.seconds
            }
        }
        simPosition += velocity * dt
    }

    // LinearSystem<# States, #Outpust, #inputs>
    fun velocitySystem(ff: SimpleMotorFeedforward): LinearSystem<N1, N1, N1> {
        if(!motorConfigured) throw MotorUnconfigured
        return LinearSystemId.identifyVelocitySystem(ff.kv, ff.ka)
    }
    fun positionSystem(ff: SimpleMotorFeedforward): LinearSystem<N2, N1, N1> {
        if(!motorConfigured) throw MotorUnconfigured
        return LinearSystemId.identifyPositionSystem(ff.kv, ff.ka)
    }
    fun dcSystem(momentOfInertia: Double): LinearSystem<N2, N1, N2> {
        if(!motorConfigured) throw MotorUnconfigured
        return LinearSystemId.createDCMotorSystem(motorType, momentOfInertia, gearRatio)
    }
    fun flywheelSystem(momentOfInertia: Double): LinearSystem<N1, N1, N1> {
        if(!motorConfigured) throw MotorUnconfigured
        return LinearSystemId.createFlywheelSystem(motorType, momentOfInertia, gearRatio)
    }
    fun elevatorSystem(mass: Double): LinearSystem<N2, N1, N1> {
        if(!motorConfigured) throw MotorUnconfigured
        return LinearSystemId.createElevatorSystem(motorType, mass, radius!!.meters, gearRatio)
    }
    fun armSystem(momentOfInertia: Double): LinearSystem<N2, N1, N1> {
        if(!motorConfigured) throw MotorUnconfigured
        return LinearSystemId.createSingleJointedArmSystem(motorType, momentOfInertia, gearRatio)
    }
    fun drivetrainSystem(ff: SimpleMotorFeedforward, kvAngular: Double, kaAngular: Double, trackWidth: Length = 2.meters): LinearSystem<N2, N2, N2> {
        if(!motorConfigured) throw MotorUnconfigured
        return LinearSystemId.identifyDrivetrainSystem(ff.kv, ff.ka, kvAngular, kaAngular, trackWidth.meters)
    }
    fun customSystem() {}  // todo: allow for generation based on formula

    var torque: Double
        get() {
            if(!motorConfigured) throw MotorUnconfigured
            return motorType!!.KtNMPerAmp * motorType!!.getCurrent(velocity.radiansPerSecond, voltage)
        }
        set(value) {
            if(!motorConfigured) throw MotorUnconfigured
            voltage = motorType!!.rOhms / (value/motorType!!.KtNMPerAmp + 1.0/motorType!!.KvRadPerSecPerVolt / motorType!!.rOhms * velocity.radiansPerSecond)
        }


    fun stateSpaceControl(loop: LinearSystemLoop<N1, N1, N1>, timeDelay: Time=0.02.seconds) {
        customControl = {
            when(controlMode) {
                ControlMode.VELOCITY -> {
                    loop.nextR = VecBuilder.fill(it.velocitySetpoint.radiansPerSecond)  // r = reference (setpoint)
                    loop.correct(VecBuilder.fill(it.velocity.radiansPerSecond))  // update with empirical
                }
                ControlMode.POSITION -> {
                    loop.nextR = VecBuilder.fill(it.positionSetpoint.radians)  // r = reference (setpoint)
                    loop.correct(VecBuilder.fill(it.position.radians))  // update with empirical
                }
                else -> log("invalid control type found", logMode = LogMode.ERROR)
            }
            loop.predict(timeDelay.seconds)  // math
            val nextVoltage = loop.getU(0)  // input
            nextVoltage
        }
        if(timeDelay != 0.02.seconds) {
            Notifier{ updateVoltage() }.startPeriodic(timeDelay.seconds)
        }
    }
}

object LinearUnconfigured : Exception("You must set the wheel radius before using linear values")
object MotorUnconfigured : Exception("You must set motor type before using linear sytems")
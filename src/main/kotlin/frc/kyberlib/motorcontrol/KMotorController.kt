package frc.kyberlib.motorcontrol

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
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
import frc.kyberlib.command.*
import frc.kyberlib.math.invertIf
import frc.kyberlib.math.sign
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.simulation.Simulatable
import frc.kyberlib.simulation.Simulation
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
    private val updateNotifier = Notifier {
//        updateValues()
        updateVoltage()
    }
    var updateRate: Time = KRobot.period.seconds  // builtin notifier system
        set(value) {
            field = value
            if (field != KRobot.period.seconds) updateNotifier.startPeriodic(value.seconds)
        }
    // ----- configs ----- //
    /**
     * Defines the relationship between rotation and linear motion for the motor.
     */
    var radius: Length? = null

    /**
     * Adds post-encoder gearing to allow for post-geared speeds to be set.
     *
     * product of (Input teeth / output teeth) for each gear stage
     */
    var gearRatio: GearRatio = 1.0
    /**
     * Settings relevant to the motor controller's encoder.
     */
    var encoderConfig: KEncoderConfig = KEncoderConfig(0, EncoderType.NONE)
        set(value) {
            if (configureEncoder(value)) field = value
            else System.err.println("Invalid encoder configuration")
        }
    // ----- constraints ---- //
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
    open var maxPosition: Angle? = null
    var maxLinearPosition: Length?
        get() = maxPosition?.let { rotationToLinear(it) }
        set(value) {maxPosition = if (value == null) value else linearToRotation(value)}
    open var minPosition: Angle? = null
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

    // ----- control schemes ---- //
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
    var PID = ProfiledPIDController(0.0, 0.0, 0.0, constraints)  // what does the profile do?

    /**
     * Builtin control that will combine feedforward with the PID.
     * Useful in both position and velocity control
     */
    fun addFeedforward(feedforward: SimpleMotorFeedforward) {
        customControl = {
            when (controlMode) {
                ControlMode.VELOCITY -> {
                    val ff = if (linearConfigured) feedforward.calculate(linearVelocitySetpoint.metersPerSecond)///, linearVelocitySetpoint.metersPerSecond, updateRate.seconds)
                                else feedforward.calculate(velocitySetpoint.radiansPerSecond)//, velocitySetpoint.radiansPerSecond, updateRate.seconds)
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
     */
    fun addFeedforward(feedforward: ArmFeedforward) {
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
                else -> 0.0
            }
        }
    }
    fun addFeedforward(feedforward: ElevatorFeedforward) {
        if (!linearConfigured) log("elevator requires linear option", logMode = LogMode.ERROR, level = DebugLevel.MaxPriority)
        customControl = {
            when (controlMode) {
                ControlMode.POSITION -> {
                    val ff = feedforward.calculate(0.0)
                    val pid = PID.calculate(linearPositionError.meters)
                    ff + pid
                }
                ControlMode.VELOCITY -> {
                    val ff = feedforward.calculate(linearVelocity.metersPerSecond)
                    val pid = PID.calculate(linearVelocityError.metersPerSecond)
                    ff + pid
                }
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
            velocity = state.velocity.radiansPerSecond
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
    var customControl: ((it: KMotorController) -> Double)? = {  // todo: figure out if prebuilt should exist
        when (controlMode) {
            ControlMode.POSITION -> PID.calculate(positionError.radians)
            ControlMode.VELOCITY -> PID.calculate(velocityError.radiansPerSecond)
            else -> 0.0
        }
    }
    /**
     * Locks recursive calls to customControl from inside of customControl
     */
    private var customControlLock = false

    // ----- motor state information ----- //
    /**
     * Angle that the motor is at / should be at
     */
    var position: Angle
        get() = if (Game.real) (rawPosition * gearRatio.invertIf { reversed }).k else simPosition
        set(value) {
            controlMode = ControlMode.POSITION
            positionSetpoint = value
        }
    /**
     * Distance the motor has traveled
     */
    var linearPosition: Length
        get() = rotationToLinear(position)
        set(value) { position = linearToRotation(value) }
    /**
     * Spin rate of motor system
     */
    var velocity: AngularVelocity
        get() = if(Game.real) rawVelocity * gearRatio.invertIf { reversed } else simVelocity
        set(value) {
            controlMode = ControlMode.VELOCITY
            velocitySetpoint = value
        }
    /**
     * Linear velocity of the motor system
     */
    var linearVelocity: LinearVelocity
        get() = rotationToLinear(velocity)
        set(value) { velocity = linearToRotation(value) }

    // ----- error ---- //
    val positionError get() = position - positionSetpoint
    val linearPositionError get() = linearPosition - linearPositionSetpoint
    val velocityError get() = velocity - velocitySetpoint
    val linearVelocityError get() = linearVelocity - linearVelocitySetpoint

    // ----- setpoints ---- //
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
            field = value.coerceIn(-maxVelocity, maxVelocity)
            if (!closedLoopConfigured && real) rawVelocity = field
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

    // ----- natives ----- //
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

    override fun initSendable(builder: NTSendableBuilder) {
        super.initSendable(builder)
        builder.setActuator(true)
        builder.setUpdateTable {
            updateVoltage()
        }
        if (linearConfigured) {
            builder.addDoubleProperty("Linear Position (m)", {linearPosition.meters}, {})
            builder.addDoubleProperty("Linear Position Setpoint (m)", {linearPositionSetpoint.meters}, { if (it.meters != linearPosition) linearPosition = it.meters })
            builder.addDoubleProperty("Linear Velocity (m per s)", {linearVelocity.metersPerSecond}, {})
            builder.addDoubleProperty("Linear Setpoint Velocity (m per s)", {linearVelocitySetpoint.metersPerSecond}, { if (it.metersPerSecond != linearVelocity) linearVelocity = it.metersPerSecond })
        } else {
            builder.addDoubleProperty("Angular Position (degrees)", {position.degrees}, {})
            builder.addDoubleProperty("Angular Position Setpoint (degrees)", {positionSetpoint.degrees}, { if (it.degrees != position) position = it.degrees })
            builder.addDoubleProperty("Angular Velocity (rad per s)", {velocity.radiansPerSecond}, {})
            builder.addDoubleProperty("Angular Velocity Setpoint (rad per s)", {velocitySetpoint.radiansPerSecond}, { if (it.radiansPerSecond != velocity) velocity = it.radiansPerSecond })
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

    // ----- sim ---- //
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

    fun setupSim(feedforward: SimpleMotorFeedforward) {
        if (Game.sim) Simulation.instance.include(this)
        simUpdater = { dt: Time -> feedforwardUpdate(feedforward.ks, feedforward.kv, feedforward.ka, dt) }
    }
    fun setupSim(feedforward: ArmFeedforward) {
        if(!brakeMode) log("Use brakeMode", logMode = LogMode.WARN)
        if (Game.sim) Simulation.instance.include(this)
        simUpdater = { dt: Time -> feedforwardUpdate(feedforward.ks + feedforward.kcos * position.cos, feedforward.kv, feedforward.ka, dt) }
    }
    fun setupSim(feedforward: ElevatorFeedforward) {
        if(!brakeMode) log("Use brakeMode", logMode = LogMode.WARN)
        if (Game.sim) Simulation.instance.include(this)
        simUpdater = {dt: Time -> feedforwardUpdate(feedforward.ks * voltage.sign + feedforward.kg, feedforward.kv, feedforward.ka, dt) }
    }
    private fun feedforwardUpdate(staticVolt: Double, kV: Double, kA: Double, dt: Time) {
        if(voltage.absoluteValue < staticVolt) {
            simVelocity = 0.radiansPerSecond
            // kinda assuming brakeMode
        }
        else {
            val applicableVolt = (voltage.absoluteValue - staticVolt).invertIf { voltage < 0.0 }
            val vel = velocity.radiansPerSecond
            val accVolt = applicableVolt - kV * vel
            val acceleration = accVolt / kA
            if (brakeMode) {
                val velMaintananceVolt = staticVolt + kV * velocity.radiansPerSecond.absoluteValue
                if (velMaintananceVolt.sign != velocity.radiansPerSecond.sign && vel != 0.0)
                    simVelocity = 0.radiansPerSecond
                else if (velMaintananceVolt < voltage.absoluteValue)
                    simVelocity = (applicableVolt / kV).radiansPerSecond
                else
                    simVelocity += acceleration.radiansPerSecond * dt.seconds
            }
            else {
                simVelocity += acceleration.radiansPerSecond * dt.seconds
            }
        }
        simPosition += velocity * dt
    }
    fun setupSim(system: LinearSystem<N1, N1, N1>) {
        val sim = LinearSystemSim(system)
        if (Game.sim) Simulation.instance.include(this)
        simUpdater = { dt: Time ->
            sim.setInput(voltage)
            sim.update(dt.seconds)
            simVelocity = sim.getOutput(0).radiansPerSecond
        }
    }
    @JvmName("setupPositionSim")
    fun setupSim(system: LinearSystem<N2, N1, N1>) {
        val sim = LinearSystemSim(system)
        if (Game.sim) Simulation.instance.include(this)
        simUpdater = { dt: Time ->
            sim.setInput(voltage)
            sim.update(dt.seconds)
            simPosition = sim.getOutput(0).radians
        }
    }
    @JvmName("setupDualSim")
    fun setupSim(system: LinearSystem<N2, N1, N2>) {
        val sim = LinearSystemSim(system)
        if (Game.sim) Simulation.instance.include(this)
        simUpdater = { dt: Time ->
            sim.setInput(voltage)
            sim.update(dt.seconds)
            simPosition = sim.getOutput(0).radians
            simVelocity = sim.getOutput(1).radiansPerSecond
        }
    }

    lateinit var simUpdater: (Time) -> Unit
    override fun simUpdate(dt: Time) {
        if(this::simUpdater.isInitialized) simUpdater(dt)
    }

    // ----- state-space ---- // -> LinearSystem<# States, #Outpust, #inputs> todo: add documentation
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
    fun armSystem(armFeedforward: ArmFeedforward): LinearSystem<N2, N1, N1> {
        if(!motorConfigured) throw MotorUnconfigured
        val motor = motorType!!
        return LinearSystem(  // todo: check if this works
            // a
            Matrix.mat(N2.instance, N2.instance).fill(0.0, 1.0, 0.0, -armFeedforward.kv/armFeedforward.ka),
            // b
            VecBuilder.fill(0.0, 1.0 / armFeedforward.ka),
            // c
            Matrix.mat(Nat.N1(), Nat.N2()).fill(1.0, 0.0),
            // d
            Matrix(Nat.N1(), Nat.N1())
        )
    }
    fun drivetrainSystem(ff: SimpleMotorFeedforward, kvAngular: Double, kaAngular: Double, trackWidth: Length = 2.meters): LinearSystem<N2, N2, N2> {
        if(!motorConfigured) throw MotorUnconfigured
        return LinearSystemId.identifyDrivetrainSystem(ff.kv, ff.ka, kvAngular, kaAngular, trackWidth.meters)
    }
    fun estimateFF(momentOfInertia: Double){
        val ka = (motorType!!.rOhms * momentOfInertia) / (gearRatio * motorType!!.KtNMPerAmp)
        val kv = -gearRatio * gearRatio * motorType!!.KtNMPerAmp / (motorType!!.KvRadPerSecPerVolt * motorType!!.rOhms * momentOfInertia) * -ka
        println("kv: $kv, ka: $ka")
    }

    var torque: Double
        get() {
            if(!motorConfigured) throw MotorUnconfigured
            return motorType!!.KtNMPerAmp * motorType!!.getCurrent(velocity.radiansPerSecond, voltage)
        }
        set(value) {
            if(!motorConfigured) throw MotorUnconfigured
            voltage = motorType!!.rOhms / (value/motorType!!.KtNMPerAmp + 1.0/motorType!!.KvRadPerSecPerVolt / motorType!!.rOhms * velocity.radiansPerSecond)
        }

    @JvmName("velocityStateSpaceControl")
    fun stateSpaceControl(loop: LinearSystemLoop<N1, N1, N1>, timeDelay: Time=0.02.seconds) {
        customControl = {
            loop.nextR = VecBuilder.fill(it.velocitySetpoint.radiansPerSecond)  // r = reference (setpoint)
            loop.correct(VecBuilder.fill(it.velocity.radiansPerSecond))  // update with empirical
            loop.predict(timeDelay.seconds)  // math
            val nextVoltage = loop.getU(0)  // input
            nextVoltage
        }
        updateRate = timeDelay
    }
    @JvmName("positionStateSpaceControl")
    fun stateSpaceControl(loop: LinearSystemLoop<N2, N1, N1>, timeDelay: Time=0.02.seconds) {
        customControl = {
            loop.nextR = VecBuilder.fill(positionSetpoint.radians, velocitySetpoint.radiansPerSecond)
            loop.correct(VecBuilder.fill(position.radians))
            loop.predict(timeDelay.seconds)  // math
            val nextVoltage = loop.getU(0)  // input
            nextVoltage
        }
        updateRate = timeDelay
    }
    @JvmName("dualStateSpaceControl")
    fun stateSpaceControl(loop: LinearSystemLoop<N2, N1, N2>, timeDelay: Time=0.02.seconds) {
        customControl = {
            loop.nextR = VecBuilder.fill(positionSetpoint.value, velocitySetpoint.value)
            loop.correct(VecBuilder.fill(position.value, velocity.value))
            loop.predict(timeDelay.seconds)  // math
            val nextVoltage = loop.getU(0)  // input
            nextVoltage
        }
        updateRate = timeDelay
    }
    @JvmName("velocityStateSpaceControl")
    fun stateSpaceControl(plant: LinearSystem<N1, N1, N1>, modelAccuracy: Double, measurementAccuracy: Double, errorCost: Double, inputCost: Double=12.0, timeDelay: Time=0.02.seconds) { stateSpaceControl(StateSpace.systemLoop(plant, modelAccuracy, measurementAccuracy, errorCost, inputCost, timeDelay)) }
    @JvmName("positionStateSpaceControl")
    fun stateSpaceControl(plant: LinearSystem<N2, N1, N1>, modelAccuracy: Double, measurementAccuracy: Double, positionErrorCost: Double, velocityErrorCost: Double, inputCost: Double=12.0, timeDelay: Time=0.02.seconds) { stateSpaceControl(StateSpace.systemLoop(plant, modelAccuracy, measurementAccuracy, positionErrorCost, velocityErrorCost, inputCost, timeDelay)) }
    @JvmName("dualStateSpaceControl")
    fun stateSpaceControl(plant: LinearSystem<N2, N1, N2>, modelAccuracy: Double, measurementAccuracy: Double, positionErrorCost: Double, velocityErrorCost: Double, inputCost: Double=12.0, timeDelay: Time=0.02.seconds) { stateSpaceControl(StateSpace.systemLoop(plant, modelAccuracy, measurementAccuracy, positionErrorCost, velocityErrorCost, inputCost, timeDelay)) }

    object StateSpace {
        @JvmName("velocityObserver")
        fun observer(plant: LinearSystem<N1, N1, N1>, modelAccuracy: Double, measurementAccuracy: Double, timeDelay: Time=0.02.seconds): KalmanFilter<N1, N1, N1> {
            return KalmanFilter(
                N1.instance, N1.instance,
                plant,
                VecBuilder.fill(modelAccuracy),  // How accurate we think our model is
                VecBuilder.fill(measurementAccuracy),  // How accurate we think our encoder
                timeDelay.seconds
            )

        }
        @JvmName("positionObserver")
        fun observer(plant: LinearSystem<N2, N1, N1>, modelAccuracy: Double, measurementAccuracy: Double, timeDelay: Time=0.02.seconds): KalmanFilter<N2, N1, N1> {
            return KalmanFilter(
                N2.instance, N1.instance,
                plant,
                VecBuilder.fill(modelAccuracy, modelAccuracy),  // How accurate we think our model is
                VecBuilder.fill(measurementAccuracy),  // How accurate we think our encoder
                timeDelay.seconds
            )

        }
        @JvmName("dualObserver")
        fun observer(plant: LinearSystem<N2, N1, N2>, modelAccuracy: Double, measurementAccuracy: Double, timeDelay: Time=0.02.seconds): KalmanFilter<N2, N1, N2> {
            return KalmanFilter(
                N2.instance, N2.instance,
                plant,
                VecBuilder.fill(modelAccuracy, modelAccuracy),  // How accurate we think our model is
                VecBuilder.fill(measurementAccuracy, measurementAccuracy),  // How accurate we think our encoder
                timeDelay.seconds
            )

        }

        @JvmName("velocityOptimizer")
        fun optimizer(plant: LinearSystem<N1, N1, N1>, velocityTolerance: Double, voltageTolerance: Double=12.0, timeDelay: Time=0.02.seconds): LinearQuadraticRegulator<N1, N1, N1> {
            return LinearQuadraticRegulator(
                plant,
                VecBuilder.fill(velocityTolerance),  // q-elms. Velocity error tolerance, in radians per second.
                VecBuilder.fill(voltageTolerance),  // r-elms. 12 cause max battery voltage
                timeDelay.seconds  // estimated loop time. 0.020 for TimedRobot, but lower if using notifiers.
            )
        }
        @JvmName("positionOptimizer")
        fun optimizer(plant: LinearSystem<N2, N1, N1>, velocityTolerance: Double, positionTolerance: Double, voltageTolerance: Double=12.0, timeDelay: Time=0.02.seconds): LinearQuadraticRegulator<N2, N1, N1> {
            return LinearQuadraticRegulator(
                plant,
                VecBuilder.fill(positionTolerance, velocityTolerance),  // q-elms. Velocity error tolerance, in radians per second.
                VecBuilder.fill(voltageTolerance),  // r-elms. 12 cause max battery voltage
                timeDelay.seconds  // estimated loop time. 0.020 for TimedRobot, but lower if using notifiers.
            )
        }
        @JvmName("dualOptimizer")
        fun optimizer(plant: LinearSystem<N2, N1, N2>, velocityTolerance: Double, positionTolerance: Double, voltageTolerance: Double=12.0, timeDelay: Time=0.02.seconds): LinearQuadraticRegulator<N2, N1, N2> {
            return LinearQuadraticRegulator(
                plant,
                VecBuilder.fill(positionTolerance, velocityTolerance),  // q-elms. Velocity error tolerance, in radians per second.
                VecBuilder.fill(voltageTolerance),  // r-elms. 12 cause max battery voltage
                timeDelay.seconds  // estimated loop time. 0.020 for TimedRobot, but lower if using notifiers.
            )
        }

        @JvmName("velocitySystemLoop")
        fun systemLoop(plant: LinearSystem<N1, N1, N1>,
                       modelAccuracy: Double, measurementAccuracy: Double,
                       velocityTolerance: Double,
                       voltageTolerance: Double=12.0,
                       timeDelay: Time=0.02.seconds): LinearSystemLoop<N1, N1, N1> {
            val kalman = observer(plant, modelAccuracy, measurementAccuracy, timeDelay)
            val lqr = optimizer(plant, velocityTolerance, voltageTolerance, timeDelay)
            return LinearSystemLoop(plant, lqr, kalman, Game.batteryVoltage, timeDelay.seconds)
        }
        @JvmName("positionSystemLoop")
        fun systemLoop(plant: LinearSystem<N2, N1, N1>,
                       modelAccuracy: Double, measurementAccuracy: Double,
                       positionErrorCost: Double, velocityErrorCost: Double,
                       inputCost: Double=12.0,
                       timeDelay: Time=0.02.seconds): LinearSystemLoop<N2, N1, N1> {
            val observer = observer(plant, modelAccuracy, measurementAccuracy, timeDelay)
            val optimizer = optimizer(plant, positionErrorCost, velocityErrorCost, inputCost, timeDelay)
            return LinearSystemLoop(plant, optimizer, observer, Game.batteryVoltage, timeDelay.seconds)
        }
        @JvmName("dualSystemLoop")
        fun systemLoop(plant: LinearSystem<N2, N1, N2>,
                       modelAccuracy: Double, measurementAccuracy: Double,
                       positionErrorCost: Double, velocityErrorCost: Double,
                       inputCost: Double=12.0,
                       timeDelay: Time=0.02.seconds): LinearSystemLoop<N2, N1, N2> {
            val observer = observer(plant, modelAccuracy, measurementAccuracy, timeDelay)
            val optimizer = optimizer(plant, positionErrorCost, velocityErrorCost, inputCost, timeDelay)
            return LinearSystemLoop(plant, optimizer, observer, Game.batteryVoltage, timeDelay.seconds)
        }
    }

    init {
        assert(encoderConfigured) {"configure your motor before using"}
    }
}

object LinearUnconfigured : Exception("You must set the wheel radius before using linear values")
object MotorUnconfigured : Exception("You must set motor type before using linear sytems")
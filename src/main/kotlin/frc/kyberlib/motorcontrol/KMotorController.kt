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
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.command.Game
import frc.kyberlib.command.KRobot
import frc.kyberlib.command.LogMode
import frc.kyberlib.math.filters.Differentiator
import frc.kyberlib.math.invertIf
import frc.kyberlib.math.sign
import frc.kyberlib.math.units.KUnit
import frc.kyberlib.math.units.KUnitKey
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.simulation.Simulatable
import frc.kyberlib.simulation.Simulation
import frc.robot.Constants
import kotlin.math.absoluteValue

typealias GearRatio = Double
typealias BrakeMode = Boolean

/**
 * Types of encoders that may be used
 */
enum class EncoderType {
    NEO_HALL, QUADRATURE
}

/**
 * The type update goal the motor is going for
 */
enum class ControlMode {
    VELOCITY, POSITION, VOLTAGE, NULL
}

/**
 * A more advanced motor control with feedback control.
 */
abstract class KMotorController : KBasicMotorController(), Simulatable {
    open var motorType: DCMotor? = null

    private val updateNotifier = Notifier { updateVoltage() }
    var updateRate: Time = KRobot.period.seconds  // builtin notifier system
        set(value) {
            field = value
            PID = PIDController(kP, kI, kD, value.seconds)
            if (field != KRobot.period.seconds) updateNotifier.startPeriodic(value.seconds)
            else updateNotifier.stop()
        }
    // ----- configs ----- //
    /**
     * Defines the relationship between rotation and linear motion for the motor.
     */
    var radius: Length? = null
        set(value) {
            field = value
            updateNativeControl(kP * toNative, kI*toNative, kD*toNative, kF+toNative)
        }

    /**
     * Adds post-encoder gearing to allow for post-geared speeds to be set.
     *
     * product of (output teeth / input teeth) for each gear stage
     */
    var gearRatio: GearRatio = 1.0
        set(value) {
            field = value
            updateNativeControl(kP * toNative, kI*toNative, kD*toNative, kF+toNative)
        }
    // ----- constraints ---- //
    /**
     * The max angular velocity the motor can have
     */
    open var maxVelocity: AngularVelocity = 0.rpm
        set(value) {
            field = value
            updateNativeProfile(maxVelocity.rotationsPerSecond * toNative, maxAcceleration.rotationsPerSecond * toNative)
        }

    /**
     * The max angular acceleration the motor can have
     */
    open var maxAcceleration: AngularVelocity = 0.rpm
        set(value) {
            field = value
            updateNativeProfile(maxVelocity.rotationsPerSecond * toNative, maxAcceleration.rotationsPerSecond * toNative)
        }

    open var maxPosition: Angle = Angle(Double.POSITIVE_INFINITY)
    var maxLinearPosition: Length
        get() = rotationToLinear(maxPosition)
        set(value) {
            maxPosition = linearToRotation(value)
        }
    open var minPosition: Angle = Angle(Double.NEGATIVE_INFINITY)
    var minLinearPosition: Length
        get() = rotationToLinear(minPosition)
        set(value) {
            minPosition = linearToRotation(value)
        }

    /**
     * The max linear velocity the motor can have
     */
    var maxLinearVelocity: LinearVelocity
        get() = rotationToLinear(maxVelocity)
        set(value) {
            maxVelocity = linearToRotation(value)
        }

    /**
     * The max linear acceleration the motor can have
     */
    var maxLinearAcceleration: LinearVelocity
        get() = rotationToLinear(maxAcceleration)
        set(value) {
            maxAcceleration = linearToRotation(value)
        }

    protected abstract fun updateNativeProfile(maxVelocity: Double? = null, maxAcceleration: Double? = null,
                                               rampRate: Double? = null)
    // ----- control schemes ---- //
    /**
     * Proportional gain of the customControl controller.
     */
    var kP: Double
        get() = PID.p
        set(value) {
            PID.p = value
            updateNativeControl(kP * toNative, kI*toNative, kD*toNative, kF+toNative)
        }

    /**
     * Integral gain of the customControl controller.
     */
    var kI: Double
        get() = PID.i
        set(value) {
            PID.i = value
            updateNativeControl(kP * toNative, kI*toNative, kD*toNative, kF+toNative)
        }

    /**
     * Derivative gain of the customControl controller.
     */
    var kD: Double
        get() = PID.d
        set(value) {
            PID.d = value
            updateNativeControl(kP * toNative, kI*toNative, kD*toNative, kF+toNative)
        }

    /**
     * FF gain for native motor control.
     * Should be equal to kv on SysID ff
     */
    var kF: Double = 0.0
        set(value) {
            field = value
            updateNativeControl(kP * toNative, kI*toNative, kD*toNative, kF+toNative)
        }

    private val toNative
        get() = if(linearConfigured) radius!!.meters * gearRatio else gearRatio

    protected abstract fun updateNativeControl(p: Double, i: Double, d: Double, f: Double)

    /**
     * Max integration value
     */
    var kIRange = 0.0
        set(value) {
            field = value
            PID.setIntegratorRange(-value, value)
        }

    /**
     * Builtin PID controller for motor
     */
    var PID = PIDController(0.0, 0.0, 0.0)

    /**
     * Builtin control that will combine feedforward with the PID.
     * Useful in both position and velocity control
     */
    fun addFeedforward(feedforward: SimpleMotorFeedforward) {
        kF = feedforward.kv
        customControl = {
            when (controlMode) {
                ControlMode.VELOCITY -> {
                    val ff =
                        if (linearConfigured) feedforward.calculate(linearVelocitySetpoint.metersPerSecond)///, linearVelocitySetpoint.metersPerSecond, updateRate.seconds)
                        else feedforward.calculate(velocitySetpoint.radiansPerSecond)//, velocitySetpoint.radiansPerSecond, updateRate.seconds)
                    val pid = PID.calculate(
                        velocity.radiansPerSecond,
                        velocitySetpoint.radiansPerSecond
                    )
                    ff + pid
                }
                ControlMode.POSITION -> {
                    val pid = PID.calculate(position.radians, positionSetpoint.radians)
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
                    val ff = feedforward.calculate(
                        position.radians,
                        PID.calculate(position.radians, positionSetpoint.radians)
                    )
                    ff
                }
                ControlMode.VELOCITY -> {
                    val ff = feedforward.calculate(position.radians, velocity.radiansPerSecond)
                    val pid = PID.calculate(velocity.radiansPerSecond, velocitySetpoint.radiansPerSecond)
                    ff + pid
                }
                else -> 0.0
            }
        }
    }

    /**
     * Add builtin control to control elevator from FF
     */
    fun addFeedforward(feedforward: ElevatorFeedforward) {
        if (!linearConfigured) log("elevator requires linear option", logMode = LogMode.ERROR, level = DebugFilter.Max)
        customControl = {
            when (controlMode) {
                ControlMode.POSITION -> {
                    val ff = feedforward.calculate(0.0)
                    val pid = PID.calculate(linearPosition.meters, linearPositionSetpoint.meters)
                    ff + pid
                }
                ControlMode.VELOCITY -> {
                    val ff = feedforward.calculate(linearVelocity.metersPerSecond)
                    val pid = PID.calculate(linearVelocity.metersPerSecond, linearVelocitySetpoint.metersPerSecond)
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
    var customControl: ((motor: KMotorController) -> Voltage)? = null

    /**
     * Locks recursive calls to customControl from inside customControl
     */
    private var customControlLock = false  // this could brake but I don't think its been an issue so far

    // ----- motor state information ----- //
    /**
     * Angle that the motor is at / should be at
     */
    var position: Angle
        get() = if (Game.real) (rawPosition / gearRatio.invertIf { reversed }) else simPosition
        set(value) {
            controlMode = ControlMode.POSITION
            positionSetpoint = value
        }

    /**
     * Distance the motor has traveled
     */
    var linearPosition: Length
        get() = rotationToLinear(position)
        set(value) {
            position = linearToRotation(value)
        }

    /**
     * Spin rate of motor system
     */
    var velocity: AngularVelocity
        get() = if (Game.real) rawVelocity / gearRatio.invertIf { reversed } else simVelocity
        set(value) {
            controlMode = ControlMode.VELOCITY
            velocitySetpoint = value
        }

    /**
     * Linear velocity of the motor system
     */
    var linearVelocity: LinearVelocity
        get() = rotationToLinear(velocity)
        set(value) {
            velocity = linearToRotation(value)
        }

    private val accelerationCalculator = Differentiator()
    var acceleration = 0.rpm
        protected set
    val linearAcceleration
        get() = rotationToLinear(acceleration)


    // ----- error ---- //
    /**
     * Angle between where it is and where it wants to be
     */
    inline val positionError get() = position - positionSetpoint

    /**
     * Distance between where it is and where it wants to be
     */
    inline val linearPositionError get() = linearPosition - linearPositionSetpoint

    /**
     * Velocity difference between where it is and where it wants to be
     */
    inline val velocityError get() = velocity - velocitySetpoint

    /**
     * Linear Velocity difference between where it is and where it wants to be
     */
    inline val linearVelocityError get() = linearVelocity - linearVelocitySetpoint

    // ----- setpoints ---- //
    /**
     * Sets the angle to which the motor should go
     */
    var positionSetpoint: Angle = 0.rotations
        private set(value) {
            field = value.coerceIn(minPosition, maxPosition)
            if (!closedLoopConfigured && real) rawPosition = field * gearRatio.invertIf { reversed }
            else if (!customControlLock) updateVoltage()
        }

    /**
     * Sets the velocity to which the motor should go
     */
    var velocitySetpoint: AngularVelocity = 0.rpm
        private set(value) {
            field = value//.coerceIn(-maxVelocity, maxVelocity)
            if (!closedLoopConfigured && real) rawVelocity = field * gearRatio.invertIf { reversed }
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
    @JvmName("linearToRotation1")
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
    @JvmName("rotationToLinear1")
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
            acceleration = accelerationCalculator.calculate(velocity.value).radiansPerSecond
            customControlLock = true
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
        inline get() = radius != null

    /**
     * Whether the type of DC motor has been set. Relevant for some Statespace and sim stuff
     */
    private val motorConfigured
        inline get() = motorType != null

    /**
     * Does the motor have closed-loop gains set?
     * Allows for closed-loop control methods to be used
     */
    private val closedLoopConfigured
        inline get() = customControl != null

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
    @JvmName("resetPosition1")
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
     * Creates Sendable which will enable manual control over the motors
     */
    override fun initSendable(builder: NTSendableBuilder) {
        super.initSendable(builder)
        builder.setUpdateTable {
            updateVoltage()
        }
        if (linearConfigured) {
            builder.addDoubleProperty("Linear Position (m)", { linearPosition.meters }, null)
            builder.addDoubleProperty(
                "Linear Position Setpoint (m)",
                { linearPositionSetpoint.meters },
                { if (it.meters != linearPosition) linearPosition = it.meters })
            builder.addDoubleProperty("Linear Velocity (m per s)", { linearVelocity.metersPerSecond }, null)
            builder.addDoubleProperty(
                "Linear Setpoint Velocity (m per s)",
                { linearVelocitySetpoint.metersPerSecond },
                { if (it.metersPerSecond != linearVelocity) linearVelocity = it.metersPerSecond })
        } else {
            builder.addDoubleProperty("Angular Position (degrees)", { position.degrees }, null)
            builder.addDoubleProperty(
                "Angular Position Setpoint (degrees)",
                { positionSetpoint.degrees },
                { if (it.degrees != position) position = it.degrees })
            builder.addDoubleProperty("Angular Velocity (rad per s)", { velocity.radiansPerSecond }, null)
            builder.addDoubleProperty(
                "Angular Velocity Setpoint (rad per s)",
                { velocitySetpoint.radiansPerSecond },
                { if (it.radiansPerSecond != velocity) velocity = it.radiansPerSecond })
        }
    }

    override fun debugValues(): Map<String, Any?> {
        val map = super.debugValues().toMutableMap()
        if (linearConfigured)
            map.putAll(
                mapOf(
                    "Linear Position" to linearPosition,
                    "Linear Velocity" to linearVelocity,
                    // "Linear Acceleration (m per s per s)" to linearAcceleration.metersPerSecond
                )
            )
        else
            map.putAll(
                mapOf(
                    "Angular Position" to position,
                    "Angular Velocity" to velocity,
                    // "Angular Acceleration (rad per s^2)" to acceleration.radiansPerSecond  // temporary (here for testing)
                )
            )
        if (controlMode == ControlMode.POSITION) {
            if (linearConfigured)
                map.putAll(
                    mapOf(
                        "setpoint" to linearPositionSetpoint,
                        "error" to linearPositionError
                    )
                )
            else
                map.putAll(
                    mapOf(
                        "setpoint" to positionSetpoint,
                        "error" to positionError
                    )
                )
        } else {
            if (linearConfigured)
                map.putAll(
                    mapOf(
                        "setpoint" to linearVelocitySetpoint.metersPerSecond,
                        "error" to linearVelocityError.metersPerSecond
                    )
                )
            else
                map.putAll(
                    mapOf(
                        "setpoint" to velocitySetpoint.radiansPerSecond,
                        "error" to velocityError.radiansPerSecond
                    )
                )
        }
//        map["PID"] = PID
        return map.toMap()
    }

    // ----- sim ---- //
    /**
     * Settable variable describing velocity according to whatever simulation
     */
    var simVelocity: AngularVelocity = 0.rpm
        set(value) {
            assert(!real) { "This value should only be set from a simulation" }
            field = value
            acceleration = accelerationCalculator.calculate(value.radiansPerSecond).radiansPerSecond
        }

    /**
     * Settable variable describing velocity according to whatever simulation
     */
    var simPosition: Angle = 0.degrees
        set(value) {
            assert(!real) { "This value should only be set from a simulation" }
            field = value
        }
    var simLinearVelocity
        get() = rotationToLinear(simVelocity)
        set(value) {
            simVelocity = linearToRotation(value)
        }
    var simLinearPosition
        get() = rotationToLinear(simPosition)
        set(value) {
            simPosition = linearToRotation(value)
        }

    /**
     * Setup simulations based on a feedforward of how the motor should move
     */
    fun setupSim(feedforward: SimpleMotorFeedforward) {
        if (Game.sim) Simulation.instance.include(this)
        simUpdater = { dt: Time -> feedforwardUpdate(feedforward.ks, feedforward.kv, feedforward.ka, dt) }
    }

    fun setupSim(feedforward: ArmFeedforward) {
        if (!brakeMode) log("Use brakeMode", logMode = LogMode.WARN)
        if (Game.sim) Simulation.instance.include(this)
        simUpdater = { dt: Time ->
            feedforwardUpdate(
                feedforward.ks + feedforward.kcos * position.cos,
                feedforward.kv,
                feedforward.ka,
                dt
            )
        }
    }

    fun setupSim(feedforward: ElevatorFeedforward) {
        if (!brakeMode) log("Use brakeMode", logMode = LogMode.WARN)
        if (Game.sim) Simulation.instance.include(this)
        simUpdater = { dt: Time ->
            feedforwardUpdate(
                feedforward.ks * voltage.sign + feedforward.kg,
                feedforward.kv,
                feedforward.ka,
                dt
            )
        }
    }

    /**
     * Private math to estimate how the feedforward would predict
     */
    private fun feedforwardUpdate(staticVolt: Voltage, kV: Double, kA: Double, dt: Time) {
        if (voltage.absoluteValue < staticVolt) {
            simVelocity = 0.radiansPerSecond
            // kinda assuming brakeMode
        } else {
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
            } else {
                simVelocity += acceleration.radiansPerSecond * dt.seconds
            }
        }
        simPosition += velocity * dt
    }

    /**
     * Setup simulation based on LinearSystem plant
     */
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
        if (this::simUpdater.isInitialized) simUpdater(dt)
    }

    // ----- state-space ---- // -> LinearSystem<# States, #Outpust, #inputs>
    /**
     * A note on Linear Systems.
     * Linear System use generics to ensure the linear algebra works out.
     *
     * the first input is the number of states the system has. This should be how many things you need to keep track of in order to model the system
     *
     * the second input is the number of outputs in the system. There is typically one output, voltage.
     *
     * the final input is the number of inputs to the system. These are the things you are trying to control like position
     *
     * @see positionSystem - this has 2 states (position and velocity), 1 output (voltage), and 1 input (position setpoint)
     */

    /**
     * Creates a plant that model how velocity for a motor evolves
     */
    fun velocitySystem(ff: SimpleMotorFeedforward): LinearSystem<N1, N1, N1> {
        return LinearSystemId.identifyVelocitySystem(ff.kv, ff.ka)
    }

    /**
     * Creates a plant that model how position for a motor evolves
     */
    fun positionSystem(ff: SimpleMotorFeedforward): LinearSystem<N2, N1, N1> {
        return LinearSystemId.identifyPositionSystem(ff.kv, ff.ka)
    }

    /**
     * Creates a plant that model how position and velocity for a motor evolves
     */
    fun dcSystem(momentOfInertia: Double): LinearSystem<N2, N1, N2> {
        if (!motorConfigured) throw MotorUnconfigured
        return LinearSystemId.createDCMotorSystem(motorType, momentOfInertia, gearRatio)
    }

    /**
     * Creates a plant that model how velocity for a flywheel evolves based off its moment of inertia
     */
    fun flywheelSystem(momentOfInertia: Double): LinearSystem<N1, N1, N1> {
        if (!motorConfigured) throw MotorUnconfigured
        return LinearSystemId.createFlywheelSystem(motorType, momentOfInertia, gearRatio)
    }

    /**
     * Creates a plant that model how position for an elevator system of a given mass
     */
    fun elevatorSystem(mass: Double): LinearSystem<N2, N1, N1> {
        if (!motorConfigured) throw MotorUnconfigured
        return LinearSystemId.createElevatorSystem(motorType, mass, radius!!.meters, gearRatio)
    }

    /**
     * Creates a model of the position of an arm system given the moment of interia around the joint
     */
    fun armSystem(momentOfInertia: Double): LinearSystem<N2, N1, N1> {
        if (!motorConfigured) throw MotorUnconfigured
        return LinearSystemId.createSingleJointedArmSystem(motorType, momentOfInertia, gearRatio)
    }

    /**
     * Creates model of the position of an arm system for its characterization values
     */
    fun armSystem(armFeedforward: ArmFeedforward): LinearSystem<N2, N1, N1> {
        return LinearSystem(
            // a
            Matrix.mat(N2.instance, N2.instance).fill(0.0, 1.0, 0.0, -armFeedforward.kv / armFeedforward.ka),
            // b
            VecBuilder.fill(0.0, 1.0 / armFeedforward.ka),
            // c
            Matrix.mat(Nat.N1(), Nat.N2()).fill(1.0, 0.0),
            // d
            Matrix(Nat.N1(), Nat.N1())
        )
    }

    /**
     * Creates a model to represent the two velocities of a drivetrain given ffs
     */
    fun drivetrainSystem(
        ff: SimpleMotorFeedforward,
        kvAngular: Double,
        kaAngular: Double,
        trackWidth: Length = 2.meters
    ): LinearSystem<N2, N2, N2> {
        return LinearSystemId.identifyDrivetrainSystem(ff.kv, ff.ka, kvAngular, kaAngular, trackWidth.meters)
    }

    /**
     * Set or retrieve the estimated torque the motor is generating
     * @throws MotorUnconfigured if you have not set motor type
     */
    var torque: Double
        get() {
            if (!motorConfigured) throw MotorUnconfigured
            return motorType!!.KtNMPerAmp * motorType!!.getCurrent(velocity.radiansPerSecond, voltage)
        }
        set(value) {
            if (!motorConfigured) throw MotorUnconfigured
            voltage =
                motorType!!.rOhms / (value / motorType!!.KtNMPerAmp + 1.0 / motorType!!.KvRadPerSecPerVolt / motorType!!.rOhms * velocity.radiansPerSecond)
        }

    /**
     * Sets a control system based around a velocity control loop
     */
    @JvmName("velocityStateSpaceControl")
    fun stateSpaceControl(loop: LinearSystemLoop<N1, N1, N1>, timeDelay: Time = 0.02.seconds) {
        customControl = {
            loop.nextR = VecBuilder.fill(it.velocitySetpoint.radiansPerSecond)  // r = reference (setpoint)
            loop.correct(VecBuilder.fill(it.velocity.radiansPerSecond))  // update with empirical
            loop.predict(timeDelay.seconds)  // math
            val nextVoltage = loop.getU(0)  // input
            nextVoltage
        }
        updateRate = timeDelay
    }

    /**
     * Sets a control system based around a position control loop
     */
    @JvmName("positionStateSpaceControl")
    fun stateSpaceControl(loop: LinearSystemLoop<N2, N1, N1>, timeDelay: Time = 0.02.seconds) {
        customControl = {
            loop.nextR = VecBuilder.fill(positionSetpoint.radians, velocitySetpoint.radiansPerSecond)
            loop.correct(VecBuilder.fill(position.radians))
            loop.predict(timeDelay.seconds)  // math
            val nextVoltage = loop.getU(0)  // input
            nextVoltage
        }
        updateRate = timeDelay
    }

    /**
     * Sets a control system based around a dc motor control loop
     */
    @JvmName("dualStateSpaceControl")
    fun stateSpaceControl(loop: LinearSystemLoop<N2, N1, N2>, timeDelay: Time = 0.02.seconds) {
        customControl = {
            loop.nextR = VecBuilder.fill(positionSetpoint.value, velocitySetpoint.value)
            loop.correct(VecBuilder.fill(position.value, velocity.value))
            loop.predict(timeDelay.seconds)  // math
            val nextVoltage = loop.getU(0)  // input
            nextVoltage
        }
        updateRate = timeDelay
    }
    //-----------------------------------------//
    /**
     * Sets a control system based around a system and control parameters
     * @param plant the system that models this motor
     * @param modelAccuracy how much to trust the system math should me
     * @param measurementAccuracy how much to trust the encoder values
     * @param errorCost the tolerance on how far off from the target the motor can be. Higher cost means more agressive correction.
     * @param inputCost the tolerance on the amount of voltage. Defaults to battery voltage (12.0). Higher cost will mean more hesitant to use a lot of power.
     * @param timeDelay the time between each loop. Defaults to robot periodic but other values will create a notifier for faster updates.
     */
    @JvmName("velocityStateSpaceControl")
    fun stateSpaceControl(
        plant: LinearSystem<N1, N1, N1>,
        modelAccuracy: Double,
        measurementAccuracy: Double,
        errorCost: Double,
        inputCost: Double = 12.0,
        timeDelay: Time = 0.02.seconds
    ) {
        stateSpaceControl(
            StateSpace.systemLoop(
                plant,
                modelAccuracy,
                measurementAccuracy,
                errorCost,
                inputCost,
                timeDelay
            )
        )
    }

    @JvmName("positionStateSpaceControl")
    fun stateSpaceControl(
        plant: LinearSystem<N2, N1, N1>,
        modelAccuracy: Double,
        measurementAccuracy: Double,
        positionErrorCost: Double,
        velocityErrorCost: Double,
        inputCost: Double = 12.0,
        timeDelay: Time = 0.02.seconds
    ) {
        stateSpaceControl(
            StateSpace.systemLoop(
                plant,
                modelAccuracy,
                measurementAccuracy,
                positionErrorCost,
                velocityErrorCost,
                inputCost,
                timeDelay
            )
        )
    }

    @JvmName("dualStateSpaceControl")
    fun stateSpaceControl(
        plant: LinearSystem<N2, N1, N2>,
        modelAccuracy: Double,
        measurementAccuracy: Double,
        positionErrorCost: Double,
        velocityErrorCost: Double,
        inputCost: Double = 12.0,
        timeDelay: Time = 0.02.seconds
    ) {
        stateSpaceControl(
            StateSpace.systemLoop(
                plant,
                modelAccuracy,
                measurementAccuracy,
                positionErrorCost,
                velocityErrorCost,
                inputCost,
                timeDelay
            )
        )
    }

    /**
     * Statespace utility functions
     */
    object StateSpace {
        /**
         * Creates a Kalman Filter to ensure measurement accuracy.
         *
         * A Kalman filter combines math of what should happen and noisy measurements of state to give a combined better estimates
         *
         * @param plant the system to model your motor
         * @param modelAccuracy how much you trust the system math
         * @param measurementAccuracy how much you trust the measurements from the encoder
         * @param timeDelay how long between each update
         * @return a Kalman filter combining these parameters with the same dimensions as the plant
         */

        @JvmName("velocityObserver")
        fun observer(
            plant: LinearSystem<N1, N1, N1>,
            modelAccuracy: Double,
            measurementAccuracy: Double,
            timeDelay: Time = 0.02.seconds
        ): KalmanFilter<N1, N1, N1> {
            return KalmanFilter(
                N1.instance, N1.instance,
                plant,
                VecBuilder.fill(modelAccuracy),  // How accurate we think our model is
                VecBuilder.fill(measurementAccuracy),  // How accurate we think our encoder
                timeDelay.seconds
            )

        }

        @JvmName("positionObserver")
        fun observer(
            plant: LinearSystem<N2, N1, N1>,
            modelAccuracy: Double,
            measurementAccuracy: Double,
            timeDelay: Time = 0.02.seconds
        ): KalmanFilter<N2, N1, N1> {
            return KalmanFilter(
                N2.instance, N1.instance,
                plant,
                VecBuilder.fill(modelAccuracy, modelAccuracy),  // How accurate we think our model is
                VecBuilder.fill(measurementAccuracy),  // How accurate we think our encoder
                timeDelay.seconds
            )

        }

        @JvmName("dualObserver")
        fun observer(
            plant: LinearSystem<N2, N1, N2>,
            modelAccuracy: Double,
            measurementAccuracy: Double,
            timeDelay: Time = 0.02.seconds
        ): KalmanFilter<N2, N1, N2> {
            return KalmanFilter(
                N2.instance, N2.instance,
                plant,
                VecBuilder.fill(modelAccuracy, modelAccuracy),  // How accurate we think our model is
                VecBuilder.fill(measurementAccuracy, measurementAccuracy),  // How accurate we think our encoder
                timeDelay.seconds
            )

        }

        /**
         * A Linear Quadractic Regulator takes a system and the cost of state error and voltage error and creates an optimal adjustment scheme.
         * @param plant system to model the motor
         * @param velocityTolerance how far of from the desired state is acceptables
         * @param voltageTolerance what is the max acceptable voltage. Default to battery voltage (12.0)
         * @return a LQR that will optimize the control system for your plant
         */

        @JvmName("velocityOptimizer")
        fun optimizer(
            plant: LinearSystem<N1, N1, N1>,
            velocityTolerance: Double,
            voltageTolerance: Double = 12.0,
            timeDelay: Time = 0.02.seconds
        ): LinearQuadraticRegulator<N1, N1, N1> {
            return LinearQuadraticRegulator(
                plant,
                VecBuilder.fill(velocityTolerance),  // q-elms. Velocity error tolerance, in radians per second.
                VecBuilder.fill(voltageTolerance),  // r-elms. 12 cause max battery voltage
                timeDelay.seconds  // estimated loop time. 0.020 for TimedRobot, but lower if using notifiers.
            )
        }

        @JvmName("positionOptimizer")
        fun optimizer(
            plant: LinearSystem<N2, N1, N1>,
            velocityTolerance: Double,
            positionTolerance: Double,
            voltageTolerance: Double = 12.0,
            timeDelay: Time = 0.02.seconds
        ): LinearQuadraticRegulator<N2, N1, N1> {
            return LinearQuadraticRegulator(
                plant,
                VecBuilder.fill(
                    positionTolerance,
                    velocityTolerance
                ),  // q-elms. Velocity error tolerance, in radians per second.
                VecBuilder.fill(voltageTolerance),  // r-elms. 12 cause max battery voltage
                timeDelay.seconds  // estimated loop time. 0.020 for TimedRobot, but lower if using notifiers.
            )
        }

        @JvmName("dualOptimizer")
        fun optimizer(
            plant: LinearSystem<N2, N1, N2>,
            velocityTolerance: Double,
            positionTolerance: Double,
            voltageTolerance: Double = 12.0,
            timeDelay: Time = 0.02.seconds
        ): LinearQuadraticRegulator<N2, N1, N2> {
            return LinearQuadraticRegulator(
                plant,
                VecBuilder.fill(
                    positionTolerance,
                    velocityTolerance
                ),  // q-elms. Velocity error tolerance, in radians per second.
                VecBuilder.fill(voltageTolerance),  // r-elms. 12 cause max battery voltage
                timeDelay.seconds  // estimated loop time. 0.020 for TimedRobot, but lower if using notifiers.
            )
        }

        /**
         * Generates a System Loop that is a combination of the system, kalman filer, and LQR
         *
         * The parameters are the same as the functions above.
         */
        @JvmName("velocitySystemLoop")
        fun systemLoop(
            plant: LinearSystem<N1, N1, N1>,
            modelAccuracy: Double, measurementAccuracy: Double,
            velocityTolerance: Double,
            voltageTolerance: Double = 12.0,
            timeDelay: Time = 0.02.seconds
        ): LinearSystemLoop<N1, N1, N1> {
            val kalman = observer(plant, modelAccuracy, measurementAccuracy, timeDelay)
            val lqr = optimizer(plant, velocityTolerance, voltageTolerance, timeDelay)
            return LinearSystemLoop(plant, lqr, kalman, Game.batteryVoltage, timeDelay.seconds)
        }

        @JvmName("positionSystemLoop")
        fun systemLoop(
            plant: LinearSystem<N2, N1, N1>,
            modelAccuracy: Double, measurementAccuracy: Double,
            positionTolerance: Double, velocityTolerance: Double,
            inputCost: Double = 12.0,
            timeDelay: Time = 0.02.seconds
        ): LinearSystemLoop<N2, N1, N1> {
            val observer = observer(plant, modelAccuracy, measurementAccuracy, timeDelay)
            val optimizer = optimizer(plant, positionTolerance, velocityTolerance, inputCost, timeDelay)
            return LinearSystemLoop(plant, optimizer, observer, inputCost, timeDelay.seconds)  // fixme
        }

        @JvmName("dualSystemLoop")
        fun systemLoop(
            plant: LinearSystem<N2, N1, N2>,
            modelAccuracy: Double, measurementAccuracy: Double,
            positionTolerance: Double, velocityTolerance: Double,
            inputCost: Double = 12.0,
            timeDelay: Time = 0.02.seconds
        ): LinearSystemLoop<N2, N1, N2> {
            val observer = observer(plant, modelAccuracy, measurementAccuracy, timeDelay)
            val optimizer = optimizer(plant, positionTolerance, velocityTolerance, inputCost, timeDelay)
            return LinearSystemLoop(plant, optimizer, observer, Game.batteryVoltage, timeDelay.seconds)
        }
    }
}

object LinearUnconfigured : Exception("You must set the wheel radius before using linear values")
object MotorUnconfigured : Exception("You must set motor type before using linear sytems")

/**
 * Takes a moment of intertia and estimates the ff values that would be generated
 */
internal fun estimateFF(motorType: DCMotor, gearRatio: Double, momentOfInertia: Double) {
    val ka = (motorType.rOhms * momentOfInertia) / (gearRatio * motorType.KtNMPerAmp)
    val kv =
        -gearRatio * gearRatio * motorType.KtNMPerAmp / (motorType.KvRadPerSecPerVolt * motorType.rOhms * momentOfInertia) * -ka
    println("kv: $kv, ka: $ka")
}


fun main() {
    estimateFF(DCMotor.getNeo550(1), Constants.TURRET_GEAR_RATIO, 0.123787)
}
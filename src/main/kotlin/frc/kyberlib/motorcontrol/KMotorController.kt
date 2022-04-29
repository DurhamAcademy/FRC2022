package frc.kyberlib.motorcontrol

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.*
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
import frc.kyberlib.command.KRobot
import frc.kyberlib.command.LogMode
import frc.kyberlib.math.filters.Differentiator
import frc.kyberlib.math.invertIf
import frc.kyberlib.math.sign
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.statespace.Statespace
import frc.kyberlib.simulation.Simulatable
import frc.kyberlib.simulation.Simulation
import kotlin.math.absoluteValue

typealias GearRatio = Double
typealias BrakeMode = Boolean
typealias MotorControl = ((motor: KMotorController) -> Voltage)

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
abstract class KMotorController(fake: Boolean = false) : KBasicMotorController(fake), Simulatable {
    open var motorType: DCMotor? = null

    private val updateNotifier = Notifier { updateVoltage() }
    var updateRate: Time = KRobot.period.seconds  // builtin notifier system
        set(value) {
            field = value
            if (field != KRobot.period.seconds) updateNotifier.startPeriodic(value.seconds)
            else updateNotifier.stop()
        }
    // ----- configs ----- //
    /**
     * Defines the relationship between rotation and linear motion for the motor.
     */
    var radius: Length? = null

    /**
     * Adds post-encoder gearing to allow for post-geared speeds to be set.
     *
     * product of (output teeth / input teeth) for each gear stage
     */
    var gearRatio: GearRatio = 1.0  // todo: add support for shift gearbox

    fun shiftGearbox(newGearRatio: GearRatio, newCustomControl: MotorControl?=null) {
        if(newCustomControl != null) {
            gearRatio = newGearRatio
            customControl = newCustomControl
        } else {
            val change = gearRatio / newGearRatio
            gearRatio = newGearRatio
            kP *= change
            kI *= change
            kD *= change
        }
    }

    // ----- constraints ---- //
    /**
     * The max angular velocity the motor can have
     */
    var maxVelocity: AngularVelocity = 100000.rpm
        set(value) {
            field = value
            PID.setConstraints(TrapezoidProfile.Constraints(maxVelocity.value, maxAcceleration.value))
        }
    /**
     * The max angular acceleration the motor can have
     */
    var maxAcceleration: AngularVelocity = 100000.rpm
        set(value) {
            field = value
            PID.setConstraints(TrapezoidProfile.Constraints(maxVelocity.value, maxAcceleration.value))
        }

    /**
     * Max position setpoint
     */
    open var maxPosition: Angle = Angle(Double.POSITIVE_INFINITY)
    var maxLinearPosition: Length
        get() = rotationToLinear(maxPosition)
        set(value) {
            maxPosition = linearToRotation(value)
        }

    /**
     * Min position setpoint
     */
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

    abstract var currentLimit: Int
    // ----- control schemes ---- //
    /**
     * Proportional gain of the customControl controller.
     */
    inline var kP: Double
        get() = PID.p
        set(value) {
            PID.p = value
        }

    /**
     * Integral gain of the customControl controller.
     */
    inline var kI: Double
        get() = PID.i
        set(value) {
            PID.i = value
        }

    /**
     * Derivative gain of the customControl controller.
     */
    inline var kD: Double
        inline get() = PID.d
        set(value) {
            PID.d = value
        }

    /**
     * The multiplier to get from high level units (geared) to native units (ungeared)
     */
    protected val toNative
        inline get() = if(linearConfigured) radius!!.meters * gearRatio else gearRatio

    protected abstract fun implementNativeControls(slot: Int=0)

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
    var PID = ProfiledPIDController(0.0, 0.0, 0.0, TrapezoidProfile.Constraints(Double.MAX_VALUE, Double.MAX_VALUE))
    private inline val pidVal get() = when (controlMode) {
        ControlMode.VELOCITY -> {
            if (!linearConfigured)
                PID.calculate(
                    velocity.radiansPerSecond,
                    velocitySetpoint.radiansPerSecond
                )
            else
                PID.calculate(
                    linearVelocity.metersPerSecond,
                    linearVelocitySetpoint.metersPerSecond
                )
        }
        ControlMode.POSITION -> {
            if (!linearConfigured) PID.calculate(position.radians, positionSetpoint.radians)
            else PID.calculate(linearPosition.meters, linearPositionSetpoint.meters)
        }
        else -> 0.0
    }

    /**
     * Whether to flash the PID loop onto the intergrated controller. Might allow for better performace but harder to debug and mess with.
     * ***This should only be set after configuring your PID values***
     */
    var nativeControl = false  // todo
        set(value) {
            field = value
            if(real && value) implementNativeControls()
        }
    /**
     * Builtin control that will combine feedforward with the PID.
     * Useful in both position and velocity control
     */
    fun addFeedforward(feedforward: SimpleMotorFeedforward) {
        customControl = {
            when (controlMode) {
                ControlMode.VELOCITY -> {
                    val ff =
                        if (linearConfigured) feedforward.calculate(linearVelocitySetpoint.metersPerSecond)///, linearVelocitySetpoint.metersPerSecond, updateRate.seconds)
                        else feedforward.calculate(velocitySetpoint.radiansPerSecond)//, velocitySetpoint.radiansPerSecond, updateRate.seconds)
                    ff + pidVal
                }
                ControlMode.POSITION -> {
                    pidVal + feedforward.calculate(PID.goal.velocity)
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
                        0.0
                    )
                    ff + pidVal
                }
                ControlMode.VELOCITY -> {
                    val ff = feedforward.calculate(position.radians, velocity.radiansPerSecond)
                    ff + pidVal
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
                    ff + pidVal
                }
                ControlMode.VELOCITY -> {
                    val ff = feedforward.calculate(linearVelocity.metersPerSecond)
                    ff + pidVal
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
            velocity = state.velocity.radiansPerSecond
            position = state.position.radians
            if (profile.isFinished(timer.get())) {
                timer.stop()
                customControl = prevControl
            }
            if (prevControl != null) prevControl(it) else 0.0
        }
    }

    /**
     * Implements bang bang controls
     */
    fun bangBang(positionTolerance: Angle = 1.degrees, velocityTolerance: AngularVelocity = positionTolerance.value.degreesPerSecond,
                 effort: Voltage = 12.0) {
        assert(!brakeMode) {"dont use brake mode in Bang bang"}
        customControl =  {
            if(controlMode == ControlMode.POSITION) {
                if(positionTolerance > positionError.absoluteValue) 0.0
                else if(positionError.value < 0) effort
                else -effort
            } else {
                if(velocityTolerance > velocityError.absoluteValue) 0.0
                else if(velocityError.value < 0) effort
                else -effort
            }
        }
    }
    @JvmName("bangBang1")
    /**
     * Bang bang on linear values
     */
    fun bangBang(positionTolerance: Length, velocityTolerance: LinearVelocity, effort: Voltage) {
        bangBang(linearToRotation(positionTolerance), linearToRotation(velocityTolerance), effort)
    }

    /**
     * The control function of the robot.
     * Uses the motor state to determine what voltage should be applied.
     * Can be set to null to use in-built motor control system
     */
    var customControl: ((motor: KMotorController) -> Voltage)? = { pidVal}

    /**
     * The base voltage to apply to the motor when using native controls (before pid correction)
     */
    protected val arbFFVolts: Voltage
        get() {
            customControlLock = true
            val v = if(customControl != null) customControl!!(this) else 0.0
            customControlLock = false
            return v
        }

    /**
     * Copy all the settings from another KMotor
     */
    open fun copyConfig(other: KMotorController) {
        kP = other.kP
        kI = other.kI
        kD = other.kD

//        customControl = other.customControl
        reversed = other.reversed
        currentLimit = other.currentLimit
        gearRatio = other.gearRatio
        radius = other.radius
        motorType = other.motorType
        brakeMode = other.brakeMode

        maxPosition = other.maxPosition
        minPosition = other.minPosition
        maxVelocity = other.maxVelocity
        maxAcceleration = other.maxAcceleration
    }

    /**
     * Give all settings to another KMotor
     */
    fun shareConfig(other: KMotorController) {other.copyConfig(this)}

    /**
     * Locks recursive calls to customControl from inside customControl when setting position/velocity
     */
    private var customControlLock = false  // this could brake but I don't think its been an issue so far

    // ----- motor state information ----- //
    /**
     * Angle that the motor is at / should be at
     */
    var position: Angle
        get() = if (real) (rawPosition / gearRatio.invertIf { reversed }) else simPosition
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
        get() = if (real) rawVelocity / gearRatio.invertIf { reversed } else simVelocity
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
            if(!customControlLock) updateVoltage()
        }

    /**
     * Sets the velocity to which the motor should go
     */
    var velocitySetpoint: AngularVelocity = 0.rpm
        private set(value) {
            field = value//.coerceIn(-maxVelocity, maxVelocity)
            if(!customControlLock) updateVoltage()
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
        if(nativeControl) {
            if(controlMode == ControlMode.VELOCITY) rawVelocity = velocitySetpoint * gearRatio.invertIf { reversed }
            else if(controlMode == ControlMode.POSITION) rawPosition = positionSetpoint * gearRatio.invertIf { reversed }
        }
        else if (controlMode != ControlMode.VOLTAGE) {
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
    inline val linearConfigured
        get() = radius != null

    /**
     * Whether the type of DC motor has been set. Relevant for some Statespace and sim stuff
     */
    inline val motorConfigured
        get() = motorType != null

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

    // ----- sim ---- //
    // you should never really need to touch these values unless you are making a custom sim
    /**
     * Settable variable describing velocity according to whatever simulation
     */
    private var simVelocity: AngularVelocity = 0.rpm
        set(value) {
            assert(!real) { "This value should only be set from a simulation" }
            field = value
        }

    /**
     * Settable variable describing velocity according to whatever simulation
     */
    private var simPosition: Angle = 0.degrees
        set(value) {
            assert(!real) { "This value should only be set from a simulation" }
            field = value
        }
    private var simLinearVelocity
        get() = rotationToLinear(simVelocity)
        set(value) {
            simVelocity = linearToRotation(value)
        }
    private var simLinearPosition
        get() = rotationToLinear(simPosition)
        set(value) {
            simPosition = linearToRotation(value)
        }

    /**
     * Setup simulations based on a feedforward of how the motor should move
     */
    fun setupSim(feedforward: SimpleMotorFeedforward) {
        if (!real) Simulation.include(this)
        val linearFactor = if(linearConfigured) radius!!.meters else 1.0
        simUpdater = { dt: Time -> feedforwardUpdate(feedforward.ks * linearFactor, feedforward.kv * linearFactor, feedforward.ka * linearFactor, dt) }
    }

    fun setupSim(feedforward: ArmFeedforward) {
        if (!brakeMode) log("Use brakeMode", logMode = LogMode.WARN)
        if (!real) Simulation.include(this)
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
        if (!real) Simulation.include(this)
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
        if (!real) Simulation.include(this)
        simUpdater = { dt: Time ->
            sim.setInput(voltage)
            sim.update(dt.seconds)
            simVelocity = sim.getOutput(0).radiansPerSecond
            simPosition += simVelocity * dt
        }
    }

    @JvmName("setupPositionSim")
    fun setupSim(system: LinearSystem<N2, N1, N1>) {
        val sim = LinearSystemSim(system)
        if (!real) Simulation.include(this)
        simUpdater = { dt: Time ->
            sim.setInput(voltage)
            sim.update(dt.seconds)
            val newPosition = sim.getOutput(0).radians
            simVelocity = (newPosition - simPosition) / dt
            simPosition = newPosition
        }
    }

    @JvmName("setupDualSim")
    fun setupSim(system: LinearSystem<N2, N1, N2>) {
        val sim = LinearSystemSim(system)
        if (!real) Simulation.include(this)
        simUpdater = { dt: Time ->
            sim.setInput(voltage)
            sim.update(dt.seconds)
            simPosition = sim.getOutput(0).radians
            simVelocity = sim.getOutput(1).radiansPerSecond
        }
    }

    fun setupSim() {
        if(!real) Simulation.include(this)
    }
    var simUpdater = { dt: Time ->
        if(controlMode == ControlMode.POSITION) {
            simVelocity = (positionSetpoint - simPosition) / dt
            simPosition = positionSetpoint
        } else if (controlMode == ControlMode.VELOCITY) {
            simVelocity = velocitySetpoint
            simPosition += simVelocity * dt
        }
    }
    override fun simUpdate(dt: Time) {
        simUpdater(dt)
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
        if(linearConfigured) return LinearSystemId.identifyVelocitySystem(ff.kv * radius!!.meters, ff.ka * radius!!.meters)
        return LinearSystemId.identifyVelocitySystem(ff.kv, ff.ka)
    }

    /**
     * Creates a plant that model how position for a motor evolves
     */
    fun positionSystem(ff: SimpleMotorFeedforward): LinearSystem<N2, N1, N1> {
        if(linearConfigured) return LinearSystemId.identifyPositionSystem(ff.kv * radius!!.meters, ff.ka * radius!!.meters)
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
        trackWidth: Length = 1.meters
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
            return motorType!!.KtNMPerAmp * motorType!!.getCurrent(velocity.radiansPerSecond, voltage) * gearRatio
        }
        set(value) {
            if (!motorConfigured) throw MotorUnconfigured
            voltage = motorType!!.rOhms / (value / motorType!!.KtNMPerAmp + 1.0 / motorType!!.KvRadPerSecPerVolt / motorType!!.rOhms * velocity.radiansPerSecond) / gearRatio
        }

    /**
     * The linear force (in newtons) applied by the motor
     */
    var force: Double
        get() = torque * radius!!.value
        set(value) { torque = value / radius!!.value }

    /**
     * Sets a control system based around a velocity control loop
     */
    @JvmName("velocityStateSpaceControl")
    private fun stateSpaceControl(loop: LinearSystemLoop<N1, N1, N1>, timeDelay: Time = 0.02.seconds) {
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
    private fun stateSpaceControl(loop: LinearSystemLoop<N2, N1, N1>, timeDelay: Time = 0.02.seconds) {
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
    private fun stateSpaceControl(loop: LinearSystemLoop<N2, N1, N2>, timeDelay: Time = 0.02.seconds) {
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
     * @param modelDeviation how much to trust the system math should me
     * @param errorCost the tolerance on how far off from the target the motor can be. Higher cost means more agressive correction.
     * @param inputCost the tolerance on the amount of voltage. Defaults to battery voltage (12.0). Higher cost will mean more hesitant to use a lot of power.
     * @param timeDelay the time between each loop. Defaults to robot periodic but other values will create a notifier for faster updates.
     */
    @JvmName("velocityStateSpaceControl")
    fun stateSpaceControl(plant: LinearSystem<N1, N1, N1>, modelDeviation: AngularVelocity, errorCost: AngularVelocity, inputCost: Voltage = 12.0, timeDelay: Time = 0.02.seconds) {
        stateSpaceControl(
            Statespace.systemLoop(
                plant,
                modelDeviation,
                roughVelocityAccuracy,
                errorCost,
                inputCost,
                timeDelay
            )
        )
    }

    @JvmName("positionStateSpaceControl")
    fun stateSpaceControl(plant: LinearSystem<N2, N1, N1>, modelDeviation: Angle, positionTolerance: Angle, velocityTolerance: AngularVelocity, inputCost: Double = 12.0, timeDelay: Time = 0.02.seconds) {
        stateSpaceControl(
            Statespace.systemLoop(
                plant,
                modelDeviation,
                roughPositionAccuracy,
                positionTolerance,
                velocityTolerance,
                inputCost,
                timeDelay
            )
        )
    }

    @JvmName("dualStateSpaceControl")
    fun stateSpaceControl(plant: LinearSystem<N2, N1, N2>, modelDeviation: Angle, positionTolerance: Angle, velocityTolerance: AngularVelocity, inputCost: Double = 12.0, timeDelay: Time = 0.02.seconds) {
        stateSpaceControl(
            Statespace.systemLoop(
                plant,
                modelDeviation,
                roughPositionAccuracy,
                positionTolerance,
                velocityTolerance,
                inputCost,
                timeDelay
            )
        )
    }

    // same thing as above but linear
    fun stateSpaceControl(plant: LinearSystem<N1, N1, N1>, modelDeviation: LinearVelocity, errorCost: LinearVelocity, inputCost: Voltage = 12.0, timeDelay: Time = 0.02.seconds) {stateSpaceControl(plant, linearToRotation(modelDeviation), linearToRotation(errorCost), inputCost, timeDelay)}
    fun stateSpaceControl(plant: LinearSystem<N2, N1, N1>, modelDeviation: Length, positionTolerance: Length, velocityTolerance: LinearVelocity, inputCost: Voltage = 12.0, timeDelay: Time = 0.02.seconds) { stateSpaceControl(plant, linearToRotation(modelDeviation), linearToRotation(positionTolerance), linearToRotation(velocityTolerance), inputCost, timeDelay) }
    @JvmName("dualControlL")
    fun stateSpaceControl(plant: LinearSystem<N2, N1, N2>, modelDeviation: Length, positionTolerance: Length, velocityTolerance: LinearVelocity, inputCost: Voltage = 12.0, timeDelay: Time = 0.02.seconds) { stateSpaceControl(plant, linearToRotation(modelDeviation), linearToRotation(positionTolerance), linearToRotation(velocityTolerance), inputCost, timeDelay) }

    // rough guestimates for how accurate your encoder should be for Kalman filter values
    // these values have not been tuned at all
    private inline val roughPositionAccuracy
        get() = .01.radians / gearRatio
    private inline val roughVelocityAccuracy
        get() = 0.1.radiansPerSecond / gearRatio
}

object LinearUnconfigured : Exception("You must set the wheel radius before using linear values")
object MotorUnconfigured : Exception("You must set motor type before using linear sytems")

/**
 * Takes a moment of intertia and estimates the ff values that would be generated
 */
internal fun estimateFF(motorType: DCMotor, gearRatio: Double, momentOfInertia: Double) {
    val ka = (motorType.rOhms * momentOfInertia) / (gearRatio * motorType.KtNMPerAmp)
    val kv = -gearRatio * gearRatio * motorType.KtNMPerAmp / (motorType.KvRadPerSecPerVolt * motorType.rOhms * momentOfInertia) * -ka
    println("kv: $kv, ka: $ka")
}

class MotorReader(private val motor: KMotorController) : BasicMotorReader(motor) {
    val position get() = motor.position
    val positionError get() = motor.positionError
    val posisitionSetpoint get() = motor.positionSetpoint
    val linearPosition get() = motor.linearPosition
    val linearPositionError get() = motor.linearPositionError
    val linearPositionSetpoint get() = motor.linearPositionSetpoint

    val velocity get() = motor.velocity
    val velcoityError get() = motor.velocityError
    val velocitySetpoint get() = motor.velocitySetpoint
    val linearVelocity get() = motor.linearVelocity
    val linearVelocityError get() = motor.linearVelocityError
    val linearVelocitySetpoint get() = motor.linearVelocitySetpoint

    val motorType = motor.motorType
    val gearRatio = motor.gearRatio
    val motorConfigured = motor.motorConfigured
    val linearConfigured = motor.linearConfigured
}

fun main() {
    estimateFF(DCMotor.getNeo550(1), 10.0 * (215.0 / 18.0), 0.123787)
}
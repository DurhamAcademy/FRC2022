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
    var gearRatio: GearRatio = 1.0

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
     * The max angular angularVelocity the motor can have
     */
    var maxVelocity: AngularVelocity = 100000.rpm
        set(value) {
            field = value
            pid.setConstraints(TrapezoidProfile.Constraints(maxVelocity.value, maxAcceleration.value))
        }
    /**
     * The max angular acceleration the motor can have
     */
    var maxAcceleration: AngularVelocity = 100000.rpm
        set(value) {
            field = value
            pid.setConstraints(TrapezoidProfile.Constraints(maxVelocity.value, maxAcceleration.value))
        }

    /**
     * Max angle setpoint
     */
    open var maxAngle: Angle = Angle(Double.POSITIVE_INFINITY)
    var maxDistance: Length
        get() = rotationToLinear(maxAngle)
        set(value) {
            maxAngle = linearToRotation(value)
        }

    /**
     * Min angle setpoint
     */
    open var minAngle: Angle = Angle(Double.NEGATIVE_INFINITY)
    var minDistance: Length
        get() = rotationToLinear(minAngle)
        set(value) {
            minAngle = linearToRotation(value)
        }

    /**
     * The max linear angularVelocity the motor can have
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
        get() = pid.p
        set(value) {
            pid.p = value
        }

    /**
     * Integral gain of the customControl controller.
     */
    inline var kI: Double
        get() = pid.i
        set(value) {
            pid.i = value
        }

    /**
     * Derivative gain of the customControl controller.
     */
    inline var kD: Double
        inline get() = pid.d
        set(value) {
            pid.d = value
        }

    /**
     * The multiplier to get from high level units (geared) to native units (un-geared)
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
            pid.setIntegratorRange(-value, value)
        }

    /**
     * Builtin pid controller for motor
     */
    var pid = ProfiledPIDController(0.0, 0.0, 0.0, TrapezoidProfile.Constraints(Double.MAX_VALUE, Double.MAX_VALUE))
    private inline val pidVal get() = when (controlMode) {
        ControlMode.VELOCITY -> {
            if (!linearConfigured)
                pid.calculate(
                    angularVelocity.radiansPerSecond,
                    angularVelocitySetpoint.radiansPerSecond
                )
            else
                pid.calculate(
                    linearVelocity.metersPerSecond,
                    linearVelocitySetpoint.metersPerSecond
                )
        }
        ControlMode.POSITION -> {
            if (!linearConfigured) pid.calculate(angle.radians, angleSetpoint.radians)
            else pid.calculate(distance.meters, distanceSetpoint.meters)
        }
        else -> 0.0
    }

    /**
     * Whether to flash the pid loop onto the integrated controller. Might allow for better performance but harder to debug and mess with.
     * ***This should only be set after configuring your pid values***
     */
    var nativeControl = false  // todo
        set(value) {
            field = value
            if(real && value) implementNativeControls()
        }
    /**
     * Builtin control that will combine feedforward with the pid.
     * Useful in both angle and angularVelocity control
     */
    fun addFeedforward(feedforward: SimpleMotorFeedforward) {
        customControl = {
            when (controlMode) {
                ControlMode.VELOCITY -> {
                    val ff =
                        if (linearConfigured) feedforward.calculate(linearVelocitySetpoint.metersPerSecond)
                        else feedforward.calculate(angularVelocitySetpoint.radiansPerSecond)
                    ff + pidVal
                }
                ControlMode.POSITION -> {
                    pidVal + feedforward.calculate(pid.goal.velocity)
                }
                else -> 0.0
            }
        }
    }

    /**
     * Used to create builtin functions for angular angle sheet.
     */
    fun addFeedforward(feedforward: ArmFeedforward) {
        customControl = {
            when (controlMode) {
                ControlMode.POSITION -> {
                    val ff = feedforward.calculate(
                        angle.radians,
                        0.0
                    )
                    ff + pidVal
                }
                ControlMode.VELOCITY -> {
                    val ff = feedforward.calculate(angle.radians, angularVelocity.radiansPerSecond)
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
            angularVelocity = state.velocity.radiansPerSecond
            angle = state.position.radians
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
                if(positionTolerance > angleError.absoluteValue) 0.0
                else if(angleError.value < 0) effort
                else -effort
            } else {
                if(velocityTolerance > angularVelocityError.absoluteValue) 0.0
                else if(angularVelocityError.value < 0) effort
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

        maxAngle = other.maxAngle
        minAngle = other.minAngle
        maxVelocity = other.maxVelocity
        maxAcceleration = other.maxAcceleration
    }

    /**
     * Give all settings to another KMotor
     */
    fun shareConfig(other: KMotorController) {other.copyConfig(this)}

    /**
     * Locks recursive calls to customControl from inside customControl when setting angle/angularVelocity
     */
    private var customControlLock = false  // this could break, but I don't think it's been an issue so far

    // ----- motor state information ----- //
    /**
     * Angle that the motor is at / should be at
     */
    var angle: Angle
        get() = if (real) (rawAngle / gearRatio.invertIf { reversed }) else simAngle
        set(value) {
            controlMode = ControlMode.POSITION
            angleSetpoint = value
        }

    /**
     * Distance the motor has traveled
     */
    var distance: Length
        get() = rotationToLinear(angle)
        set(value) {
            angle = linearToRotation(value)
        }

    /**
     * Spin rate of motor system
     */
    var angularVelocity: AngularVelocity
        get() = if (real) rawAngularVelocity / gearRatio.invertIf { reversed } else simAngularVelocity
        set(value) {
            controlMode = ControlMode.VELOCITY
            angularVelocitySetpoint = value
        }

    /**
     * Linear velocity of the motor system
     */
    var linearVelocity: LinearVelocity
        get() = rotationToLinear(angularVelocity)
        set(value) {
            angularVelocity = linearToRotation(value)
        }

    // ----- error ---- //
    /**
     * Angle between where it is and where it wants to be
     */
    inline val angleError get() = angle - angleSetpoint

    /**
     * Distance between where it is and where it wants to be
     */
    inline val distanceError get() = distance - distanceSetpoint

    /**
     * Velocity difference between where it is and where it wants to be
     */
    inline val angularVelocityError get() = angularVelocity - angularVelocitySetpoint

    /**
     * Linear Velocity difference between where it is and where it wants to be
     */
    inline val linearVelocityError get() = linearVelocity - linearVelocitySetpoint

    // ----- setpoints ---- //
    /**
     * Sets the angle to which the motor should go
     */
    var angleSetpoint: Angle = 0.rotations
        private set(value) {
            field = value.coerceIn(minAngle, maxAngle)
            if(!customControlLock) updateVoltage()
        }

    /**
     * Sets the angularVelocity to which the motor should go
     */
    var angularVelocitySetpoint: AngularVelocity = 0.rpm
        private set(value) {
            field = value//.coerceIn(-maxVelocity, maxVelocity)
            if(!customControlLock) updateVoltage()
        }

    /**
     * Sets the linear angle to which the motor should go
     */
    val distanceSetpoint: Length
        get() = rotationToLinear(angleSetpoint)

    /**
     * Sets the linear angularVelocity to which the motor should go
     */
    val linearVelocitySetpoint: LinearVelocity
        get() = rotationToLinear(angularVelocitySetpoint)

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
     * Updates the voltage after changing angle / angularVelocity setpoint
     */
    fun updateVoltage() {
        if(nativeControl) {
            if(controlMode == ControlMode.VELOCITY) rawAngularVelocity = angularVelocitySetpoint * gearRatio.invertIf { reversed }
            else if(controlMode == ControlMode.POSITION) rawAngle = angleSetpoint * gearRatio.invertIf { reversed }
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
    abstract fun resetPosition(angle: Angle = 0.rotations)

    /**
     * Resets where the encoder thinks it is.
     * Does *not* move motor to that spot, just internal variable.
     */
    @JvmName("resetPosition1")
    fun resetPosition(distance: Length) {
        resetPosition(linearToRotation(distance))
    }

    /**
     * The native motors get and set methods for angle.
     * Recommend using Position instead because it is safer.
     */
    abstract var rawAngle: Angle
        protected set

    /**
     * Native motor get and set for angular angularVelocity.
     * Recommend using Velocity instead because it is safer
     */
    abstract var rawAngularVelocity: AngularVelocity
        protected set

    /**
     * Creates Sendable which will enable manual control over the motors
     */
    override fun initSendable(builder: NTSendableBuilder) {
        super.initSendable(builder)
        if (linearConfigured) {
            builder.addDoubleProperty("Distance (m)", { distance.meters }, null)
            builder.addDoubleProperty(
                "Distance Setpoint (m)",
                { distanceSetpoint.meters },
                { if (it.meters != distance) distance = it.meters })
            builder.addDoubleProperty("Linear Velocity (m per s)", { linearVelocity.metersPerSecond }, null)
            builder.addDoubleProperty(
                "Linear Setpoint Velocity (m per s)",
                { linearVelocitySetpoint.metersPerSecond },
                { if (it.metersPerSecond != linearVelocity) linearVelocity = it.metersPerSecond })
        } else {
            builder.addDoubleProperty("Angle (degrees)", { angle.degrees }, null)
            builder.addDoubleProperty(
                "Angle Setpoint (degrees)",
                { angleSetpoint.degrees },
                { if (it.degrees != angle) angle = it.degrees })
            builder.addDoubleProperty("Angular Velocity (rad per s)", { angularVelocity.radiansPerSecond }, null)
            builder.addDoubleProperty(
                "Angular Velocity Setpoint (rad per s)",
                { angularVelocitySetpoint.radiansPerSecond },
                { if (it.radiansPerSecond != angularVelocity) angularVelocity = it.radiansPerSecond })
        }
    }

    // ----- sim ---- //
    // you should never really need to touch these values unless you are making a custom sim
    /**
     * Settable variable describing angularVelocity according to whatever simulation
     */
    private var simAngularVelocity: AngularVelocity = 0.rpm
        set(value) {
            assert(!real) { "This value should only be set from a simulation" }
            field = value
        }

    /**
     * Settable variable describing angularVelocity according to whatever simulation
     */
    private var simAngle: Angle = 0.degrees
        set(value) {
            assert(!real) { "This value should only be set from a simulation" }
            field = value
        }
    private var simLinearVelocity
        get() = rotationToLinear(simAngularVelocity)
        set(value) {
            simAngularVelocity = linearToRotation(value)
        }
    private var simDistance
        get() = rotationToLinear(simAngle)
        set(value) {
            simAngle = linearToRotation(value)
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
                feedforward.ks + feedforward.kcos * angle.cos,
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
    private fun feedforwardUpdate(staticVolt: Voltage, kV: Double, kA: Double, dt: Time) {  // todO: maybe remove and just use linear systems
        if (voltage.absoluteValue < staticVolt) {
            simAngularVelocity = 0.radiansPerSecond
            // kinda assuming brakeMode
        } else {
            val applicableVolt = (voltage.absoluteValue - staticVolt).invertIf { voltage < 0.0 }
            val vel = angularVelocity.radiansPerSecond
            val accVolt = applicableVolt - kV * vel
            val acceleration = accVolt / kA
            if (brakeMode) {
                val velMaintenanceVolt = staticVolt + kV * angularVelocity.radiansPerSecond.absoluteValue
                if (velMaintenanceVolt.sign != angularVelocity.radiansPerSecond.sign && vel != 0.0)
                    simAngularVelocity = 0.radiansPerSecond
                else if (velMaintenanceVolt < voltage.absoluteValue)
                    simAngularVelocity = (applicableVolt / kV).radiansPerSecond
                else
                    simAngularVelocity += acceleration.radiansPerSecond * dt.seconds
            } else {
                simAngularVelocity += acceleration.radiansPerSecond * dt.seconds
            }
        }
        simAngle += angularVelocity * dt
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
            simAngularVelocity = sim.getOutput(0).radiansPerSecond
            simAngle += simAngularVelocity * dt
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
            simAngularVelocity = (newPosition - simAngle) / dt
            simAngle = newPosition
        }
    }

    @JvmName("setupDualSim")
    fun setupSim(system: LinearSystem<N2, N1, N2>) {
        val sim = LinearSystemSim(system)
        if (!real) Simulation.include(this)
        simUpdater = { dt: Time ->
            sim.setInput(voltage)
            sim.update(dt.seconds)
            simAngle = sim.getOutput(0).radians
            simAngularVelocity = sim.getOutput(1).radiansPerSecond
        }
    }

    fun setupSim() {
        if(!real) Simulation.include(this)
    }
    var simUpdater = { dt: Time ->
        if(controlMode == ControlMode.POSITION) {
            simAngularVelocity = (angleSetpoint - simAngle) / dt
            simAngle = angleSetpoint
        } else if (controlMode == ControlMode.VELOCITY) {
            simAngularVelocity = angularVelocitySetpoint
            simAngle += simAngularVelocity * dt
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
     * the final input is the number of inputs to the system. These are the things you are trying to control like angle
     *
     * @see positionSystem - this has 2 states (angle and angularVelocity), 1 output (voltage), and 1 input (angle setpoint)
     */

    /**
     * Creates a plant that model how angularVelocity for a motor evolves
     */
    fun velocitySystem(ff: SimpleMotorFeedforward): LinearSystem<N1, N1, N1> {
        if(linearConfigured) return LinearSystemId.identifyVelocitySystem(ff.kv * radius!!.meters, ff.ka * radius!!.meters)
        return LinearSystemId.identifyVelocitySystem(ff.kv, ff.ka)
    }

    /**
     * Creates a plant that model how angle for a motor evolves
     */
    fun positionSystem(ff: SimpleMotorFeedforward): LinearSystem<N2, N1, N1> {
        if(linearConfigured) return LinearSystemId.identifyPositionSystem(ff.kv * radius!!.meters, ff.ka * radius!!.meters)
        return LinearSystemId.identifyPositionSystem(ff.kv, ff.ka)
    }

    /**
     * Creates a plant that model how angle and angularVelocity for a motor evolves
     */
    fun dcSystem(momentOfInertia: Double): LinearSystem<N2, N1, N2> {
        if (!motorConfigured) throw MotorUnconfigured
        return LinearSystemId.createDCMotorSystem(motorType, momentOfInertia, gearRatio)
    }

    /**
     * Creates a plant that model how angularVelocity for a flywheel evolves based off its moment of inertia
     */
    fun flywheelSystem(momentOfInertia: Double): LinearSystem<N1, N1, N1> {
        if (!motorConfigured) throw MotorUnconfigured
        return LinearSystemId.createFlywheelSystem(motorType, momentOfInertia, gearRatio)
    }

    /**
     * Creates a plant that model how angle for an elevator system of a given mass
     */
    fun elevatorSystem(mass: Double): LinearSystem<N2, N1, N1> {
        if (!motorConfigured) throw MotorUnconfigured
        return LinearSystemId.createElevatorSystem(motorType, mass, radius!!.meters, gearRatio)
    }

    /**
     * Creates a model of the angle of an arm system given the moment of inertia around the joint
     */
    fun armSystem(momentOfInertia: Double): LinearSystem<N2, N1, N1> {
        if (!motorConfigured) throw MotorUnconfigured
        return LinearSystemId.createSingleJointedArmSystem(motorType, momentOfInertia, gearRatio)
    }

    /**
     * Creates model of the angle of an arm system for its characterization values
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
            return motorType!!.KtNMPerAmp * current * gearRatio
        }
        set(value) {
            if (!motorConfigured) throw MotorUnconfigured
            voltage = motorType!!.rOhms / (value / motorType!!.KtNMPerAmp + 1.0 / motorType!!.KvRadPerSecPerVolt / motorType!!.rOhms * angularVelocity.radiansPerSecond) / gearRatio
        }

    /**
     * The linear force (in newtons) applied by the motor
     */
    var force: Double  // fixme
        get() = torque * radius!!.value
        set(value) { torque = value / radius!!.value }

    abstract var current: Double
    val resistance
        get() = voltage / current

    /**
     * Sets a control system based around a angularVelocity control loop
     */
    @JvmName("velocityStateSpaceControl")
    private fun stateSpaceControl(loop: LinearSystemLoop<N1, N1, N1>, timeDelay: Time = 0.02.seconds) {
        customControl = {
            loop.nextR = VecBuilder.fill(it.angularVelocitySetpoint.radiansPerSecond)  // r = reference (setpoint)
            loop.correct(VecBuilder.fill(it.angularVelocity.radiansPerSecond))  // update with empirical
            loop.predict(timeDelay.seconds)  // math
            val nextVoltage = loop.getU(0)  // input
            nextVoltage
        }
        updateRate = timeDelay
    }

    /**
     * Sets a control system based around an angle control loop
     */
    @JvmName("positionStateSpaceControl")
    private fun stateSpaceControl(loop: LinearSystemLoop<N2, N1, N1>, timeDelay: Time = 0.02.seconds) {
        customControl = {
            loop.nextR = VecBuilder.fill(angleSetpoint.radians, angularVelocitySetpoint.radiansPerSecond)
            loop.correct(VecBuilder.fill(angle.radians))
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
            loop.nextR = VecBuilder.fill(angleSetpoint.value, angularVelocitySetpoint.value)
            loop.correct(VecBuilder.fill(angle.value, angularVelocity.value))
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
     * @param errorCost the tolerance on how far off from the target the motor can be. Higher cost means more aggressive correction.
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

    // rough guesstimates for how accurate your encoder should be for Kalman filter values
    // these values have not been tuned at all
    private inline val roughPositionAccuracy
        get() = .01.radians / gearRatio
    private inline val roughVelocityAccuracy
        get() = 0.1.radiansPerSecond / gearRatio
}

object LinearUnconfigured : Exception("You must set the wheel radius before using linear values")
object MotorUnconfigured : Exception("You must set motor type before using linear systems")

/**
 * Takes a moment of inertia and estimates the ff values that would be generated
 */
internal fun estimateFF(motorType: DCMotor, gearRatio: Double, momentOfInertia: Double) {
    val ka = (motorType.rOhms * momentOfInertia) / (gearRatio * motorType.KtNMPerAmp)
    val kv = -gearRatio * gearRatio * motorType.KtNMPerAmp / (motorType.KvRadPerSecPerVolt * motorType.rOhms * momentOfInertia) * -ka
    println("kv: $kv, ka: $ka")
}

class MotorReader(private val motor: KMotorController) : BasicMotorReader(motor) {
    val angle get() = motor.angle
    val angleError get() = motor.angleError
    val angleSetpoint get() = motor.angleSetpoint
    val distance get() = motor.distance
    val distanceError get() = motor.distanceError
    val distanceSetpoint get() = motor.distanceSetpoint

    val angularVelocity get() = motor.angularVelocity
    val angularVelocityError get() = motor.angularVelocityError
    val angularVelocitySetpoint get() = motor.angularVelocitySetpoint
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
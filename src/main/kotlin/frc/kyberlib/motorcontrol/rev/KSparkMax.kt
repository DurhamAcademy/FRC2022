package frc.kyberlib.motorcontrol.rev

import com.revrobotics.*
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import frc.kyberlib.math.invertIf
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.*
import frc.kyberlib.motorcontrol.EncoderType


/**
 * Represents a REV Robotics Spark MAX motor controller. Recommend using .apply to setup move configs
 * [canId] is the controller's ID on the CAN bus
 * [brushType] is the type of motor being driven. WARNING: If set incorrectly this can seriously damage hardware. You've been warned.
 */
class KSparkMax(
    private val canId: CANId,
    brushedMotor: Boolean = false,
    type: EncoderType = EncoderType.NEO_HALL,
    cpr: Int = 42
) : KMotorController() {
    // ----- low-level stuff ----- //
    override var identifier: String = CANRegistry.filterValues { it == canId }.keys.firstOrNull() ?: "can$canId"

    val spark = CANSparkMax(canId, if (brushedMotor) MotorType.kBrushed else MotorType.kBrushless)

    init {
        if (real) spark.restoreFactoryDefaults()
        CANRegistry[identifier] = canId
    }

    private var encoder: RelativeEncoder = when {
        type == EncoderType.NEO_HALL && !brushedMotor -> {
            spark.encoder
        }
        type == EncoderType.QUADRATURE && brushedMotor -> {
            spark.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, cpr)
        }
        else -> throw NotImplementedError("idk how to set your encoder values")
    }
    private val _pid = spark.pidController

    override var minPosition: Angle = super.minPosition
        set(value) {
            field = value
            if (real) {
                val direction =
                    if (reversed) CANSparkMax.SoftLimitDirection.kForward else CANSparkMax.SoftLimitDirection.kReverse
                spark.setSoftLimit(direction, (value.rotations * gearRatio.invertIf { reversed }).toFloat())
            }
        }

    override var maxPosition: Angle = super.maxPosition
        set(value) {
            field = value
            if (real) {
                val direction =
                    if (reversed) CANSparkMax.SoftLimitDirection.kReverse else CANSparkMax.SoftLimitDirection.kForward
                spark.setSoftLimit(direction, (value.rotations * gearRatio.invertIf { reversed }).toFloat())
            }
        }

    override fun checkError(): Boolean = if (real) spark.getFault(CANSparkMax.FaultID.kCANRX) else false

    override var brakeMode = false
        get() = if (real) spark.idleMode == CANSparkMax.IdleMode.kBrake else field
        set(value) {
            if (real) spark.idleMode = if (value) CANSparkMax.IdleMode.kBrake else CANSparkMax.IdleMode.kCoast
            else field = value
        }

    override var rawPercent
        get() = spark.appliedOutput
        set(value) {
            spark.set(value)
        }

    //    private val velCalc = Differentiator()
    override var rawVelocity: AngularVelocity
        get() = encoder.velocity.rpm//velCalc.calculate(rawPosition.radians).radiansPerSecond//encoder!!.velocity.rpm
        set(value) {
            _pid.setReference(
                value.rpm,
                CANSparkMax.ControlType.kVelocity,
                0,
                0.0,
                SparkMaxPIDController.ArbFFUnits.kVoltage
            )
        }

    override var rawPosition: Angle
        get() = encoder.position.rotations
        set(value) {
            _pid?.setReference(
                value.rotations,
                CANSparkMax.ControlType.kPosition,
                0,
                0.0,
                SparkMaxPIDController.ArbFFUnits.kVoltage
            )
        }

    override fun updateNativeControl(p: Double, i: Double, d: Double, f: Double) {
        _pid.p = p
        _pid.i = i
        _pid.d = d
        _pid.ff = f
    }

    override fun updateNativeProfile(maxVelocity: Double?, maxAcceleration: Double?, rampRate: Double?) {

    }

    override var maxVelocity = -1.rpm
        set(value) {
            field = value
            _pid.setSmartMotionMaxVelocity(value.rpm, 0)
        }

    override var maxAcceleration = -1.rpm
        set(value) {
            field = value
            _pid.setSmartMotionMaxAccel(value.rpm, 0)
        }

    var currentLimit: Int = -1
        set(value) {
            spark.setSmartCurrentLimit(value)
            field = value
        }

    val current
        get() = spark.outputCurrent

    override fun followTarget(kmc: KBasicMotorController) {
//        velocityRefreshRate = 100.milliseconds
//        positionRefreshRate = 100.milliseconds
        if (kmc is KSparkMax && real) {
            spark.follow(kmc.spark, reversed)
//            kmc.errorRefreshRate = followPeriodic
        } else {
            kmc.followers.add(this)
        }
    }

    override fun resetPosition(position: Angle) {
        encoder.position = position.rotations
    }

    /**
     * How quickly the spark send updates on voltage and errors
     */
    var errorRefreshRate = 10.milliseconds
        set(value) {
            field = value
            if (real) spark.setPeriodicFramePeriod(
                CANSparkMaxLowLevel.PeriodicFrame.kStatus0,
                (value.milliseconds).toInt()
            )
        }

    /**
     * How quickly the spark send update on velocity and some other stuff
     */
    var velocityRefreshRate = 20.milliseconds
        set(value) {
            field = value
            if (real) spark.setPeriodicFramePeriod(
                CANSparkMaxLowLevel.PeriodicFrame.kStatus1,
                (value.milliseconds).toInt()
            )
        }

    /**
     * How quickly the spark sends updates on position
     */
    var positionRefreshRate = 20.milliseconds
        set(value) {
            field = value
            if (real) spark.setPeriodicFramePeriod(
                CANSparkMaxLowLevel.PeriodicFrame.kStatus2,
                (value.milliseconds).toInt()
            )
        }
}

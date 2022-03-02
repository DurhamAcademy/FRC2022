package frc.kyberlib.motorcontrol.rev

import com.revrobotics.*
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import frc.kyberlib.command.LogMode
import frc.kyberlib.math.invertIf
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.milli
import frc.kyberlib.motorcontrol.*
import frc.kyberlib.motorcontrol.BrushType.BRUSHED
import frc.kyberlib.motorcontrol.BrushType.BRUSHLESS
import frc.kyberlib.motorcontrol.EncoderType
import kotlin.time.Duration.Companion.milliseconds


/**
 * Represents a REV Robotics Spark MAX motor controller. Recommend using .apply to setup move configs
 * [canId] is the controller's ID on the CAN bus
 * [brushType] is the type of motor being driven. WARNING: If set incorrectly this can seriously damage hardware. You've been warned.
 */
class KSparkMax(private val canId: CANId, private val brushType: BrushType = BRUSHLESS, type: EncoderType = EncoderType.NEO_HALL, cpr: Int = 42) : KMotorController() {
    // ----- low-level stuff ----- //
    override var identifier: String = CANRegistry.filterValues { it == canId }.keys.firstOrNull() ?: "can$canId"

    private val _spark = CANSparkMax(canId, when (brushType) {
        BRUSHLESS -> MotorType.kBrushless
        BRUSHED -> MotorType.kBrushed
    })

    init {
        if (real) _spark.restoreFactoryDefaults()
    }
    private var _enc: RelativeEncoder = when {
        type == EncoderType.NEO_HALL && brushType == BRUSHLESS -> {
            _spark.encoder
        }
        type == EncoderType.QUADRATURE && brushType == BRUSHED -> {
            _spark.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, cpr)
        }
        else -> throw NotImplementedError("idk how to set your encoder values")
    }
    private val _pid = _spark.pidController

    // todo: think about moving functionality back here for speed
    // todo: check this works
    override var minPosition: Angle? = null
        set(value) {
            field = value
            if (value != null && real) {
                val direction = if (reversed) CANSparkMax.SoftLimitDirection.kForward else CANSparkMax.SoftLimitDirection.kReverse
                _spark.setSoftLimit(direction, (value.rotations * gearRatio.invertIf { reversed }).toFloat())
            }
        }

    override var maxPosition: Angle? = null
        set(value) {
            field = value
            if (value != null && real) {
                val direction = if (reversed) CANSparkMax.SoftLimitDirection.kReverse else CANSparkMax.SoftLimitDirection.kForward
                _spark.setSoftLimit(direction, (value.rotations * gearRatio.invertIf { reversed }).toFloat())
            }
        }

    init {
        if (real) _spark.restoreFactoryDefaults()
    }

    override fun checkError(): Boolean = if(real) _spark.getFault(CANSparkMax.FaultID.kCANTX) else false

    override var brakeMode = false
        get() = if(real) _spark.idleMode == CANSparkMax.IdleMode.kBrake else field
        set(value) {
            if (real) _spark.idleMode = if(value) CANSparkMax.IdleMode.kBrake else CANSparkMax.IdleMode.kCoast
            else field = value
        }

    override var rawPercent
        get() = _spark.appliedOutput
        set(value) {_spark.set(value)}

    override var rawReversed: Boolean
        get() = _spark.inverted
        set(value) { _spark.inverted = value }

//    private val velCalc = Differentiator()
    override var rawVelocity: AngularVelocity
        get() = _enc.velocity.rpm//velCalc.calculate(rawPosition.radians).radiansPerSecond//_enc!!.velocity.rpm
        set(value) {
            _pid?.setReference(value.rpm, CANSparkMax.ControlType.kVelocity, 0, 0.0, SparkMaxPIDController.ArbFFUnits.kVoltage)
        }

    override var rawPosition: Angle
        get() = _enc.position.rotations
        set(value) {
            _pid?.setReference(value.rotations, CANSparkMax.ControlType.kPosition, 0, 0.0, SparkMaxPIDController.ArbFFUnits.kVoltage)
        }

    override fun stop() {
        if (real) _spark.stopMotor()
        else simVelocity = 0.rpm
    }

    override var currentLimit: Int = -1
        set(value) {
            _spark.setSmartCurrentLimit(value)
            field = value
        }

    val current
        get() = _spark.outputCurrent

    override fun followTarget(kmc: KBasicMotorController) {
        velocityRefreshRate = 100.milli.seconds
        positionRefreshRate = 100.milli.seconds
        if (kmc is KSparkMax && real) {
            _spark.follow(kmc._spark, reversed)
            kmc.errorRefreshRate = followPeriodic
        } else {
            log("Following other devices may slow down robot", logMode = logMode.Warn)
            kmc.followers.add(this)
            kmc.notifier.startPeriodic(followPeriodic.seconds)
        }
    }

    override fun resetPosition(position: Angle) {
        _enc.position = position.rotations
    }

    // todo: replace with kyberlib time
    var errorRefreshRate = 10.milli.seconds
        set(value) {
            field = value
            if (real) _spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, (value.seconds/1000).toInt())
        }

    var velocityRefreshRate = 20.milli.seconds
        set(value) {
            field = value
            if (real) _spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, (value.seconds/1000).toInt())
        }

    var positionRefreshRate = 20.milli.seconds
        set(value) {
            field = value
            if(real) _spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, (value.seconds/1000).toInt())
        }
}

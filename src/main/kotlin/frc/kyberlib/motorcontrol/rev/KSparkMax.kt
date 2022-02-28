package frc.kyberlib.motorcontrol.rev

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.RelativeEncoder
import com.revrobotics.SparkMaxPIDController
import com.revrobotics.SparkMaxRelativeEncoder
import frc.kyberlib.command.LogMode
import frc.kyberlib.math.filters.Differentiator
import frc.kyberlib.math.invertIf
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.*
import frc.kyberlib.motorcontrol.BrushType.BRUSHED
import frc.kyberlib.motorcontrol.BrushType.BRUSHLESS


/**
 * Represents a REV Robotics Spark MAX motor controller. Recommend using .apply to setup move configs
 * [canId] is the controller's ID on the CAN bus
 * [brushType] is the type of motor being driven. WARNING: If set incorrectly this can seriously damage hardware. You've been warned.
 */
class KSparkMax(private val canId: CANId, private val brushType: BrushType = BRUSHLESS) : KMotorController() {

    // ----- low-level stuff ----- //
    public override var identifier: String = CANRegistry.filterValues { it == canId }.keys.firstOrNull() ?: "can$canId"

    private val _spark = if (real) CANSparkMax(canId, when (brushType) {
        BRUSHLESS -> MotorType.kBrushless
        BRUSHED -> MotorType.kBrushed
    }) else null
    private var _enc: RelativeEncoder? = null
    private val _pid = _spark?.pidController

    // todo: check this works
    override var minPosition: Angle? = null
        set(value) {
            field = value
            if (value != null && _spark != null) {
                val direction = if (reversed) CANSparkMax.SoftLimitDirection.kForward else CANSparkMax.SoftLimitDirection.kReverse
                _spark.setSoftLimit(direction, (value.rotations * gearRatio.invertIf { reversed }).toFloat())
            }
        }

    override var maxPosition: Angle? = null
        set(value) {
            field = value
            if (value != null && _spark != null) {
                val direction = if (reversed) CANSparkMax.SoftLimitDirection.kReverse else CANSparkMax.SoftLimitDirection.kForward
                _spark.setSoftLimit(direction, (value.rotations * gearRatio.invertIf { reversed }).toFloat())
            }
        }

    init {
        if (real)
           _spark!!.restoreFactoryDefaults()
        // running NEO with integrated encoder
        if (brushType == BRUSHLESS) {
            encoderConfig = KEncoderConfig(42, EncoderType.NEO_HALL)
        }
    }

    override fun checkError(): Boolean {
        // check setCANTimeout
        return false//_spark!!.stickyFaults
    }

    override var brakeMode = false
        get() = if(real) _spark!!.idleMode == CANSparkMax.IdleMode.kBrake else field
        set(value) {
            if (real)
                _spark!!.idleMode = if(value) CANSparkMax.IdleMode.kBrake else CANSparkMax.IdleMode.kCoast
            else field = value
        }

    override var rawPercent
        get() = _spark!!.appliedOutput
        set(value) {_spark!!.set(value)}

    override var rawReversed: Boolean
        get() = _spark!!.inverted
        set(value) { _spark?.inverted = value }

//    private val velCalc = Differentiator()
    override var rawVelocity: AngularVelocity
        get() = _enc!!.velocity.rpm//velCalc.calculate(rawPosition.radians).radiansPerSecond//_enc!!.velocity.rpm
        set(value) {
            _pid!!.setReference(value.rpm, CANSparkMax.ControlType.kVelocity, 0, 0.0, SparkMaxPIDController.ArbFFUnits.kVoltage)
        }

    override var rawPosition: Angle
        get() = _enc!!.position.rotations
        set(value) {
            _pid!!.setReference(value.rotations, CANSparkMax.ControlType.kPosition, 0, 0.0, SparkMaxPIDController.ArbFFUnits.kVoltage)
        }

    override fun stop() {
        if (real)
            _spark!!.stopMotor()
        else simVelocity = 0.rpm
    }

    override var currentLimit: Int = -1
        set(value) {
            _spark?.setSmartCurrentLimit(value)
            field = value
        }

    val current
        get() = _spark?.outputCurrent

    override fun configureEncoder(config: KEncoderConfig): Boolean {
        return when {
            config.type == EncoderType.NEO_HALL && brushType == BRUSHLESS -> {
                _enc = _spark?.encoder
                true
            }
            config.type == EncoderType.QUADRATURE && brushType == BRUSHED -> {
                _enc = _spark?.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, config.cpr)
                _enc?.inverted = config.reversed
                true
            }
            else -> {
                false
            }
        }
    }

    override fun followTarget(kmc: KBasicMotorController) {
        if (kmc is KSparkMax && real) {
            _spark?.follow(kmc._spark, reversed)
        } else {
            kmc.followers.add(this)
            kmc.notifier.startPeriodic(followPeriodic.seconds)
        }
    }

    override fun resetPosition(position: Angle) {
        if (!encoderConfigured) {
            return log("Cannot reset encoder angle without configured encoder", LogMode.ERROR)
        }
        _enc?.position = position.rotations
    }
}

package frc.kyberlib.motorcontrol.rev

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.SparkMaxPIDController
import com.revrobotics.SparkMaxRelativeEncoder
import com.revrobotics.RelativeEncoder
import frc.kyberlib.command.LogMode
import frc.kyberlib.motorcontrol.EncoderType
import frc.kyberlib.motorcontrol.KEncoderConfig
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.motorcontrol.MotorType.BRUSHED
import frc.kyberlib.motorcontrol.MotorType.BRUSHLESS
import frc.kyberlib.math.units.extensions.Angle
import frc.kyberlib.math.units.extensions.AngularVelocity
import frc.kyberlib.math.units.extensions.rotations
import frc.kyberlib.math.units.extensions.rpm
import frc.kyberlib.motorcontrol.CANId
import frc.kyberlib.motorcontrol.CANRegistry
import frc.kyberlib.motorcontrol.KBasicMotorController


/**
 * Represents a REV Robotics Spark MAX motor controller. Recommend using .apply to setup move configs
 * [canId] is the controller's ID on the CAN bus
 * [motorType] is the type of motor being driven. WARNING: If set incorrectly this can seriously damage hardware. You've been warned.
 */
class KSparkMax(val canId: CANId, val motorType: frc.kyberlib.motorcontrol.MotorType = BRUSHLESS) : KMotorController() {

    // ----- low-level stuff ----- //
    public override var identifier: String = CANRegistry.filterValues { it == canId }.keys.firstOrNull() ?: "can$canId"

    private val _spark = if (real) CANSparkMax(canId, when (motorType) {
        BRUSHLESS -> MotorType.kBrushless
        BRUSHED -> MotorType.kBrushed
    }) else null
    private var _enc: RelativeEncoder? = null
    private val _pid = _spark?.pidController

    init {
        if (real)
           _spark!!.restoreFactoryDefaults()
        // running NEO with integrated encoder
        if (motorType == BRUSHLESS) {
            encoderConfig = KEncoderConfig(42, EncoderType.NEO_HALL)
        }
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

    override var rawVelocity: AngularVelocity
        get() = _enc!!.velocity.rpm
        set(value) {
            _pid!!.setReference(value.rpm, CANSparkMax.ControlType.kVelocity, 0, 0.0, SparkMaxPIDController.ArbFFUnits.kVoltage)
        }

    override var rawPosition: Angle
        get() = _enc!!.position.rotations
        set(value) {
            _pid!!.setReference(value.rotations, CANSparkMax.ControlType.kPosition, 0, 0.0, SparkMaxPIDController.ArbFFUnits.kVoltage)
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
            config.type == EncoderType.NEO_HALL && motorType == BRUSHLESS -> {
                _enc = _spark?.encoder
                true
            }
            config.type == EncoderType.QUADRATURE && motorType == BRUSHED -> {
                _enc = _spark?.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, config.cpr)
                _enc?.inverted = config.reversed
                true
            }
            else -> {
                false
            }
        }
    }

    override fun writePid(p: Double, i: Double, d: Double) {
        _pid?.p = p
        _pid?.i = i
        _pid?.d = d
    }

    override fun writeMultipler(mv: Double, mp: Double) {
        _enc?.velocityConversionFactor = mv
        _enc?.positionConversionFactor = mp
    }

    override fun followTarget(kmc: KBasicMotorController) {
        if (kmc is KSparkMax && real) {
            _spark?.follow(kmc._spark, reversed)
        } else {
            kmc.followers.add(this)
            kmc.notifier.startPeriodic(0.005)
        }
    }

    override fun resetPosition(position: Angle) {
        if (!encoderConfigured) {
            return log("Cannot reset encoder position without configured encoder", LogMode.ERROR)
        }
        _enc?.position = position.rotations
    }
}

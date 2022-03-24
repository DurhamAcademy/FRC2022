package frc.kyberlib.motorcontrol.ctre

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.VictorSPX
import frc.kyberlib.command.LogMode
import frc.kyberlib.motorcontrol.*

/**
 * Kotlin wrapper for VictorSPX on CAN
 */
class KVictorSPX(val canId: CANId) : KBasicMotorController() {
    val victor = VictorSPX(canId)

    override var identifier = CANRegistry.filterValues { it == canId }.keys.firstOrNull() ?: "can$canId"
    init {
        CANRegistry[identifier] = canId
    }

    override var brakeMode: BrakeMode = false
        set(value) {
            field = value
            victor.setNeutralMode(if(value) NeutralMode.Brake else NeutralMode.Coast)
        }

    override var rawPercent: Double
        get() = victor.motorOutputPercent
        set(value) {victor.set(ControlMode.PercentOutput, value)}

    override fun followTarget(kmc: KBasicMotorController) {
        when (kmc) {
            is KVictorSPX -> victor.follow(kmc.victor)
            is KTalon -> victor.follow(kmc.talon)
            else -> {
                log("Following other devices may slow down robot", logMode = LogMode.WARN)
                kmc.followers.add(this)
            }
        }
    }
}

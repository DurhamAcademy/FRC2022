package frc.kyberlib.motorcontrol.ctre

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.VictorSPX
import frc.kyberlib.command.LogMode
import frc.kyberlib.motorcontrol.*

/**
 * Kotlin wrapper for VictorSPX on CAN
 */
class KVictorSPX(val canId: Int) : KBasicMotorController() {
    val victor = VictorSPX(canId)

    override var identifier = "can$canId"
    override var brakeMode: BrakeMode = false
        set(value) {
            field = value
            victor.setNeutralMode(if(value) NeutralMode.Brake else NeutralMode.Coast)
        }

    override var rawPercent: Double
        get() = victor.motorOutputPercent
        set(value) {victor.set(ControlMode.PercentOutput, value)}

    init {

    }

    override fun followTarget(kmc: KBasicMotorController) {
        if(reversed) victor.inverted = true  // this may not work with none CTRE devices
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

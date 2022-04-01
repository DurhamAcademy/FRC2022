package frc.kyberlib.motorcontrol.ctre

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FollowerType
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.BaseTalon
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.BrakeMode
import frc.kyberlib.motorcontrol.CANRegistry
import frc.kyberlib.motorcontrol.KBasicMotorController
import frc.kyberlib.motorcontrol.KMotorController

class KTalon(port: Int, model: String = "Talon FX", private val unitsPerRotation: Int = 2048) : KMotorController() {
    val talon = BaseTalon(port, model)
    override var identifier: String = "$model ($port)"

    init {
        CANRegistry[identifier] = port
    }

    override fun resetPosition(position: Angle) {
        talon.selectedSensorPosition = position.degrees
    }

    override var rawPosition: Angle
        get() = talon.selectedSensorPosition.rotations / unitsPerRotation.toDouble()
        set(value) {
            talon.set(ControlMode.Position, value.rotations / unitsPerRotation)
        }
    override var rawVelocity: AngularVelocity
        get() = talon.selectedSensorVelocity.rotationsPerSecond * unitsPerRotation.toDouble()
        set(value) {
            talon.set(ControlMode.Velocity, value.rotationsPerSecond * 10 * unitsPerRotation)
        }
    override var brakeMode: BrakeMode = false
        set(value) {
            field = value
            talon.setNeutralMode(if (value) NeutralMode.Brake else NeutralMode.Brake)
        }
    override var rawPercent: Double
        get() = talon.motorOutputPercent
        set(value) {
            talon.set(ControlMode.PercentOutput, value)
        }

    val current
        get() = talon.supplyCurrent

    override fun followTarget(kmc: KBasicMotorController) {
        if (reversed) talon.inverted = true
        when (kmc) {
            is KTalon -> talon.follow(kmc.talon, FollowerType.PercentOutput)
            is KVictorSPX -> talon.follow(kmc.victor)
            else -> kmc.followers.add(this)
        }
    }


}
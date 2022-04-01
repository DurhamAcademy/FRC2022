package frc.kyberlib.motorcontrol.ctre

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.ctre.phoenix.music.Orchestra
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.Angle
import frc.kyberlib.math.units.extensions.AngularVelocity
import frc.kyberlib.math.units.extensions.rotations
import frc.kyberlib.math.units.extensions.rotationsPerSecond
import frc.kyberlib.motorcontrol.BrakeMode
import frc.kyberlib.motorcontrol.KBasicMotorController
import frc.kyberlib.motorcontrol.KMotorController


class KTalon(port: Int, model: String = "Talon FX", private val unitsPerRotation: Int = 2048) : KMotorController() {
    companion object {
        private val orchestra = Orchestra()
        // fortnite default dance
        // imperial march
        // dun dun dun
        // reveal video song from 2018
        // exotic my kid trash

        fun loadSong(file: String) {
            orchestra.loadMusic(file)
        }

        fun pauseMusic() {
            orchestra.play()
        }

        fun resumeMusic() {
            orchestra.pause()
        }
    }

    val talon = TalonFX(port, model)
    override var identifier: String = "$model ($port)"

    init {
        orchestra.addInstrument(talon)
        if (Game.real && false) {
            val kTimeoutMs = 100
            talon.configVoltageCompSaturation(12.0, 100)
            talon.enableVoltageCompensation(true);

            talon.clearMotionProfileHasUnderrun(kTimeoutMs)
            talon.clearMotionProfileTrajectories()

            talon.clearStickyFaults(kTimeoutMs)

            talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, kTimeoutMs)
            talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, kTimeoutMs)

            talon.configNominalOutputForward(0.0, kTimeoutMs)
            talon.configNominalOutputReverse(0.0, kTimeoutMs)

            talon.configPeakOutputForward(1.0, kTimeoutMs)
            talon.configPeakOutputReverse(-1.0, kTimeoutMs)

            talon.inverted = false

            talon.selectProfileSlot(0, 0)
        }
    }

    override fun resetPosition(position: Angle) {
        talon.selectedSensorPosition = position.rotations * unitsPerRotation
    }

    override var rawPosition: Angle
        get() = (talon.selectedSensorPosition / unitsPerRotation.toDouble()).rotations
        set(value) {
            talon.set(TalonFXControlMode.Position, value.rotations / unitsPerRotation)
        }
    override var rawVelocity: AngularVelocity
        get() = (talon.selectedSensorVelocity / unitsPerRotation.toDouble() * 10.0).rotationsPerSecond
        set(value) {
            talon.set(TalonFXControlMode.Velocity, value.rotationsPerSecond * 10 * unitsPerRotation, DemandType.ArbitraryFeedForward, arbFF(this)/vbus)
        }
    override var brakeMode: BrakeMode = false
        set(value) {
            field = value
            talon.setNeutralMode(if (value) NeutralMode.Brake else NeutralMode.Coast)
        }
    override var rawPercent: Double
        get() = talon.motorOutputPercent
        set(value) {
            talon.set(TalonFXControlMode.PercentOutput, value)
        }

    var current
        get() = talon.supplyCurrent
        set(value) {
            talon.set(TalonFXControlMode.Current, value)
        }

    var currentLimit = 0
        set(value) {
            field = value
            talon.configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, value.toDouble(), value + 10.0, .1))
        }

    val arbFF: (it: KTalon)->Double = { it: KTalon -> 0.0 }

    override fun updateNativeControl(p: Double, i: Double, d: Double, f: Double) {
        talon.config_kP(0, p)
        talon.config_kI(0, i)
        talon.config_kD(0, d)
        talon.config_kF(0, f)
    }

    override fun updateNativeProfile(maxVelocity: Double?, maxAcceleration: Double?, rampRate: Double?) {}

    var hertz = 0.0
        set(value) {
            field = value
            talon.set(ControlMode.MusicTone, value)
        }

    override fun followTarget(kmc: KBasicMotorController) {
        if (reversed) talon.inverted = true
        when (kmc) {
            is KTalon -> talon.follow(kmc.talon, FollowerType.PercentOutput)
            is KVictorSPX -> talon.follow(kmc.victor)
            else -> kmc.followers.add(this)
        }
    }


}
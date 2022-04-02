package frc.kyberlib.motorcontrol.ctre

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.*
import com.ctre.phoenix.music.Orchestra
import edu.wpi.first.wpilibj.motorcontrol.Talon
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.BrakeMode
import frc.kyberlib.motorcontrol.KBasicMotorController
import frc.kyberlib.motorcontrol.KMotorController


class KTalon(port: Int, model: String = "Talon FX", private val unitsPerRotation: Int = 2048) : KMotorController() {
    companion object {
        private val defaultConfig = TalonFXConfiguration().apply {
            statorCurrLimit = StatorCurrentLimitConfiguration(false, 40.0, 50.0, 0.1)
            supplyCurrLimit = SupplyCurrentLimitConfiguration(false, 40.0, 50.0, 0.1)

            openloopRamp = 0.1
            closedloopRamp = 0.1
            // voltage limits
            peakOutputForward = 1.0
            peakOutputReverse = 0.0
            nominalOutputForward = 0.0
            nominalOutputReverse = 0.0
//                    neutralDeadband = 0.001
//            voltageCompSaturation = 12.0  // note: check that this could have issues with the way voltage works in KMotorController
            // position limits
            forwardSoftLimitThreshold = 1000.0
            reverseSoftLimitThreshold = 1000.0
            forwardSoftLimitEnable = false
            reverseSoftLimitEnable = false

            // set pid sensors
            primaryPID = TalonFXPIDSetConfiguration()
            auxiliaryPID = TalonFXPIDSetConfiguration()

            // pid slots - this has values
            slot0 = SlotConfiguration()
            slot1 = SlotConfiguration()
            slot2 = SlotConfiguration()
            slot3 = SlotConfiguration()

            // motion magic profile info
            motionCruiseVelocity = 2048 * 10.0   // units: encoder per 100ms
            motionAcceleration = 2048 * 10.0
//                    motionCurveStrength = 2  // something about profile smoothing
//                    motionProfileTrajectoryPeriod = 20  // period in ms

            feedbackNotContinuous = true
            remoteSensorClosedLoopDisableNeutralOnLOS = true  // disable on disconnect

            // something about limit switches?
            clearPositionOnLimitF = false
            clearPositionOnLimitR = false
            clearPositionOnQuadIdx = false
            limitSwitchDisableNeutralOnLOS = false
            softLimitDisableNeutralOnLOS = false
        }
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
        if (Game.real && false) {
            orchestra.addInstrument(talon)
            val kTimeoutMs = 100
//            talon.configVoltageCompSaturation(12.0, 100)
            talon.enableVoltageCompensation(false)  // todo: look into if we want this

            talon.clearMotionProfileHasUnderrun(kTimeoutMs)
            talon.clearMotionProfileTrajectories()

            talon.clearStickyFaults(kTimeoutMs)
            talon.inverted = false

            talon.selectProfileSlot(0, 0)
            talon.configAllSettings(defaultConfig)
        }
    }

    override fun resetPosition(position: Angle) {
        talon.selectedSensorPosition = position.rotations * unitsPerRotation
    }

    override var rawPosition: Angle
        get() = (talon.selectedSensorPosition / unitsPerRotation.toDouble()).rotations
        set(value) {
            talon.set(ControlMode.Position, value.rotations / unitsPerRotation)
        }
    override var rawVelocity: AngularVelocity
        get() = talon.selectedSensorVelocity.falconSpeed
        set(value) {
            talon.set(ControlMode.Velocity, value.falconSpeed, DemandType.ArbitraryFeedForward, arbFF(this)/vbus)
        }
    override var brakeMode: BrakeMode = false
        set(value) {
            field = value
            talon.setNeutralMode(if (value) NeutralMode.Brake else NeutralMode.Coast)
        }
    override var rawPercent: Double
        get() = talon.motorOutputPercent
        set(value) {
            talon.set(ControlMode.PercentOutput, value)
        }

    var current
        get() = talon.supplyCurrent
        set(value) {
            talon.set(ControlMode.Current, value)
        }

    var currentLimit = 0
        set(value) {
            field = value
            talon.configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, value.toDouble(), value + 10.0, .1), 100)
        }

    var arbFF: (it: KTalon)->Double = { it: KTalon -> 0.0 }

    override fun updateNativeControl(p: Double, i: Double, d: Double, f: Double) {
        talon.config_kP(0, p)
        talon.config_kI(0, i)
        talon.config_kD(0, d)
        talon.config_kF(0, f)
    }

    override fun updateNativeProfile(maxVelocity: AngularVelocity, maxAcceleration: AngularVelocity) {
        talon.configMotionCruiseVelocity(maxVelocity.falconSpeed)
        talon.configMotionAcceleration(maxAcceleration.falconSpeed)
    }

    var hertz = 0.0
        set(value) {
            field = value
            talon.set(ControlMode.MusicTone, value)
        }

    override fun followTarget(kmc: KBasicMotorController) {
        if (reversed) talon.inverted = true // this may not work with none CTRE devices
        when (kmc) {
            is KTalon -> talon.follow(kmc.talon, FollowerType.PercentOutput)
            is KVictorSPX -> talon.follow(kmc.victor)
            else -> kmc.followers.add(this)
        }
    }


}
package frc.kyberlib.motorcontrol.ctre

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration
import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration
import com.ctre.phoenix.music.Orchestra
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.Angle
import frc.kyberlib.math.units.extensions.AngularVelocity
import frc.kyberlib.math.units.extensions.falconSpeed
import frc.kyberlib.math.units.extensions.rotations
import frc.kyberlib.motorcontrol.BrakeMode
import frc.kyberlib.motorcontrol.KBasicMotorController
import frc.kyberlib.motorcontrol.KMotorController


class KTalon(port: Int, model: String = "Talon FX", private val unitsPerRotation: Int = 2048) : KMotorController() {
    companion object {
        // https://github.dev/Team254/FRC-2020-Public/blob/master/src/main/java/com/team254/frc2020/planners/DriveMotionPlanner.java#L225
        private val defaultConfig = TalonFXConfiguration().apply {
            statorCurrLimit = StatorCurrentLimitConfiguration(false, 20.0, 60.0, .2)
            supplyCurrLimit = SupplyCurrentLimitConfiguration(false, 20.0, 60.0, .2)

            openloopRamp = 0.3
            closedloopRamp = 0.3
            // voltage limits
            peakOutputForward = 1.0
            peakOutputReverse = -1.0
            nominalOutputForward = 0.0
            nominalOutputReverse = 0.0
            neutralDeadband = 0.04
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

            // https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html?highlight=velocity#velocity-measurement-filter
            velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms  // vel = [p(t) - p(t-100ms)] / 100ms
            velocityMeasurementWindow = 64  // number of samples

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
            orchestra.loadMusic("music/$file")
        }

        fun pauseMusic() {
            orchestra.play()
        }

        fun resumeMusic() {
            orchestra.pause()
        }
    }

    val talon = TalonFX(port)
    override var identifier: String = "$model ($port)"

    init {
//        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
//        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20)
//        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000)
//        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 1000)
//        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000);

        if (Game.real) {
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
        get() {
            SmartDashboard.putNumber("falcon speed", talon.selectedSensorVelocity)
            return talon.selectedSensorVelocity.falconSpeed
        }
        set(value) {
            SmartDashboard.putNumber("set falcon speed", value.falconSpeed)
            talon.set(ControlMode.Velocity, value.falconSpeed, DemandType.ArbitraryFeedForward, arbFF(this) / vbus)
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
            talon.configSupplyCurrentLimit(
                SupplyCurrentLimitConfiguration(true, value.toDouble(), value + 10.0, .1),
                100
            )
        }

    var arbFF: (it: KTalon) -> Double = { 0.0 }

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
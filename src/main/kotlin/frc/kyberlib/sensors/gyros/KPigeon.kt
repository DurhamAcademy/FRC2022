package frc.kyberlib.sensors.gyros

import com.ctre.phoenix.sensors.PigeonIMU
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.Angle
import frc.kyberlib.math.units.extensions.AngularVelocity
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.degreesPerSecond
import frc.kyberlib.math.units.string

class KPigeon(port: Int) : KGyro() {
    val internal = if (Game.real) PigeonIMU(port) else null
    private val rawAngleStore = DoubleArray(3)
    private val pitchYawRoll: DoubleArray
        get() {
            internal?.getYawPitchRoll(rawAngleStore)
            return rawAngleStore
        }

    private val rawRateStore = DoubleArray(3)
    private val fullRate: DoubleArray
        get() {
            internal?.getRawGyro(rawRateStore)
            return rawRateStore
        }


    /**
     * How far leaned forward or back the robot is leaning
     */
    override val pitch get() = pitchYawRoll[1].degrees

    /**
     * Turn angle
     */
    override val yaw get() = if(real) internal!!.fusedHeading.degrees else sim

    /**
     * How far on the side the robot is
     */
    override val roll get() = pitchYawRoll[2].degrees

    override val pitchRate: AngularVelocity get() = fullRate[0].degreesPerSecond
    override val yawRate: AngularVelocity get() = fullRate[1].degreesPerSecond
    override val rollRate: AngularVelocity get() = fullRate[2].degreesPerSecond

    private var sim = 0.degrees
    override fun reset(angle: Angle) {
//        println("reset to ${angle.string()}")
        sim = angle
        internal?.fusedHeading = angle.degrees
    }
}
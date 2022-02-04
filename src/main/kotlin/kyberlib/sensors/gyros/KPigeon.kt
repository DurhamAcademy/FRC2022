package kyberlib.sensors.gyros

import com.ctre.phoenix.sensors.PigeonIMU
import kyberlib.command.Game
import kyberlib.math.units.extensions.KRotation
import kyberlib.math.units.extensions.degrees

class KPigeon(port: Int) :  KGyro {
    val internal = if (Game.real) PigeonIMU(port) else null

    override var heading: KRotation = 0.degrees
        get() =
            if (real) internal!!.fusedHeading.degrees
            else field
        set(value) {
            if (real) internal!!.fusedHeading = value.degrees
            else field = value
        }

    private val pitchYawRoll: DoubleArray
        get() {
            val store = DoubleArray(3)
            internal?.getYawPitchRoll(store)
            return store
        }

    /**
     * How far leaned forward or back the robot is leaning
     */
    val pitch get() = pitchYawRoll[1].degrees

    /**
     * Turn angle
     */
    val yaw get() = heading

    /**
     * How far on the side the robot is
     */
    val roll get() = pitchYawRoll[2].degrees
}
package kyberlib.sensors.gyros

import com.ctre.phoenix.sensors.PigeonIMU
import kyberlib.math.units.extensions.KRotation
import kyberlib.math.units.extensions.degrees
import kyberlib.simulation.Simulation

class KPigeon(port: Int) :  KGyro {
    val internal = if (Simulation.real) PigeonIMU(port) else null

    override var heading: KRotation = 0.degrees
        get() =
            if (real) internal!!.fusedHeading.degrees
            else field
        set(value) {
            if (real) internal!!.fusedHeading = value.degrees
            else field = value
        }
}
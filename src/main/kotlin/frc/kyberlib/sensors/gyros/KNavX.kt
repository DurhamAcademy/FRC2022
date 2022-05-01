package frc.kyberlib.sensors.gyros

import com.ctre.phoenix.sensors.PigeonIMU
//import com.kauailabs.navx.frc.AHRS
import frc.kyberlib.math.units.extensions.Angle
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.normalized

class KNavX : KPigeon(0) {  // fixme
//    val internal = PigeonIMU()//AHR()

//    private var offset = 0.degrees

//    override var heading: Angle
//        get() = (internal.fusedHeading.degrees - offset).normalized
//        set(value) {offset = value}
}
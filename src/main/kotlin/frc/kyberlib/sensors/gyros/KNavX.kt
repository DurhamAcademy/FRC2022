package frc.kyberlib.sensors.gyros

import com.kauailabs.navx.frc.AHRS
import frc.kyberlib.math.units.extensions.Angle
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.normalized

class KNavX : KGyro {
    val internal = AHRS()

    private var offset = 0.degrees

    override var heading: Angle
        get() = (internal.fusedHeading.degrees - offset).normalized
        set(value) {offset = value}
}
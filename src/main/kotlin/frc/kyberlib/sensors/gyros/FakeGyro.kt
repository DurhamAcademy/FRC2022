package frc.kyberlib.sensors.gyros

import frc.kyberlib.math.units.extensions.Angle
import frc.kyberlib.math.units.extensions.degrees

class FakeGyro : KGyro {
    override var heading: Angle = 0.degrees
}
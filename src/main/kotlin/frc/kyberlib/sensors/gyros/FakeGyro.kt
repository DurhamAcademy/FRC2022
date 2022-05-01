package frc.kyberlib.sensors.gyros

import frc.kyberlib.math.units.extensions.Angle
import frc.kyberlib.math.units.extensions.AngularVelocity
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.radiansPerSecond

class FakeGyro : KGyro() {
    override val pitch: Angle = 0.degrees
    override val yaw: Angle
        get() = angle
    override val roll: Angle = 0.degrees
    override val pitchRate: AngularVelocity = 0.radiansPerSecond
    override val yawRate: AngularVelocity = 0.radiansPerSecond
    override val rollRate: AngularVelocity = 0.radiansPerSecond

    private var angle = 0.degrees

    override fun reset(angle: Angle) {
        this.angle = angle
    }
}
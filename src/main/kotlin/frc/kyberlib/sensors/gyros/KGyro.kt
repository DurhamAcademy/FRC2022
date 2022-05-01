package frc.kyberlib.sensors.gyros

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.interfaces.Gyro
import frc.kyberlib.math.units.extensions.Angle
import frc.kyberlib.math.units.extensions.AngularVelocity

/**
 * A gyro wrapper that can be added to any gyro
 */
abstract class KGyro {
    /** Whether the Robot is real or simulation */
    val real = RobotBase.isReal()

    abstract val pitch: Angle
    abstract val yaw: Angle
    abstract val roll: Angle
    abstract val pitchRate: AngularVelocity
    abstract val yawRate: AngularVelocity
    abstract val rollRate: AngularVelocity

    abstract fun reset(angle: Angle)
    /**
     * The KRotation that the gyro is heading
     */
    inline val heading: Angle
        get() = yaw

    /**
     * How fast the gyro is spinning
     */
    inline val rate: AngularVelocity
        get() = yawRate
}
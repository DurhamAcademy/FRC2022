package frc.kyberlib.sensors.gyros

import edu.wpi.first.wpilibj.RobotBase
import frc.kyberlib.math.units.extensions.Angle

/**
 * A gyro wrapper that can be added to any gyro
 */
interface KGyro {
    /** Whether the Robot is real or simulation */
    val real
        get() = RobotBase.isReal()

    /**
     * The KRotation that the gyro is heading
     */
    var heading: Angle
}
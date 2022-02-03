package kyberlib.sensors.gyros

import edu.wpi.first.wpilibj.RobotBase
import kyberlib.math.units.extensions.KRotation
import edu.wpi.first.wpilibj.interfaces.Gyro

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
    var heading: KRotation
}
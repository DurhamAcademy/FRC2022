package frc.kyberlib.motorcontrol

import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Servo
import frc.kyberlib.command.DebugLevel
import frc.kyberlib.command.LogMode
import frc.kyberlib.math.units.extensions.*
import java.lang.UnsupportedOperationException
import kotlin.math.absoluteValue

// in 2022: this is for https://www.gobilda.com/2000-series-dual-mode-servo-25-2-torque/
/**
 * Handles a velocity servo
 */
class KServo(port: Int) : KMotorController() {
    private val native = Servo(port)
    private var accumulatedPosition = 0.degrees

    override fun resetPosition(position: Angle) {
        accumulatedPosition = position
    }

    init {
        if(real) {
            log("Remember this notifier exists")
            Notifier {
                accumulatedPosition += rawVelocity * 0.02.seconds
            }.startPeriodic(0.02)
        }
    }

    override var minPosition: Angle? = 0.degrees
    override var maxPosition: Angle? = 300.degrees
    override var rawPosition: Angle
        get() = if(controlMode == ControlMode.POSITION) accumulatedPosition else accumulatedPosition
        set(value) {
            native.position = value.degrees / maxPosition!!.degrees
        }
    override var rawVelocity: AngularVelocity = 0.radiansPerSecond

    override var brakeMode: BrakeMode = true
    override var rawReversed: Boolean = false
    override var currentLimit: Int = -1
    override var identifier: String = "Servo$port"
    override var rawPercent: Double
        get() = native.get()
        set(value) {
            native.set(value)
            for(follower in followers) follower.percent = value  // todo: maybe implement this as the default follow
        }

    override fun followTarget(kmc: KBasicMotorController) {
        if(kmc is KServo) {
            kmc.followers.add(this)
        } else throw UnsupportedOperationException("This hasn't been made yet")
    }
}
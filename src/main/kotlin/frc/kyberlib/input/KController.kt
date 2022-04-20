package frc.kyberlib.input

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick

abstract class KController(port: Int = 0) {
    companion object {
        var sim = false
    }
    protected val joystick = Joystick(port)

    var rumbleLeft = 0.0
        set(value) {
            joystick.setRumble(GenericHID.RumbleType.kLeftRumble, value)
            field = value
        }
    var rumbleRight = 0.0
        set(value) {
            joystick.setRumble(GenericHID.RumbleType.kRightRumble, value)
            field = value
        }
    var rumble: Double
        get() = rumbleLeft.coerceAtLeast(rumbleRight)
        set(value) {
            rumbleLeft = value
            rumbleRight = value
        }
}

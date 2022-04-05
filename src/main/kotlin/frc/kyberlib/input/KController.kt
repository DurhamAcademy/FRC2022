package frc.kyberlib.input

import edu.wpi.first.wpilibj.Joystick

abstract class KController(port: Int = 0) {
    companion object {
        var sim = false
    }
    protected val joystick = Joystick(port)
}

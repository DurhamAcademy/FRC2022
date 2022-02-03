package kyberlib.input

import edu.wpi.first.wpilibj.Joystick

abstract class KController(port: Int = 0) {
    protected val joystick = Joystick(port)
}

package frc.kyberlib.input.controller

import frc.kyberlib.input.KAxis
import frc.kyberlib.input.KController


/**
 * Wrapper for a Taranis controller using TAER on channels 1-4
 */
class KTaranis(port: Int) : KController(port) {
    val throttle = KAxis { joystick.getRawAxis(0) }
    val aileron = KAxis { joystick.getRawAxis(1) }
    val elevator = KAxis { joystick.getRawAxis(2) }
    val rudder = KAxis { joystick.getRawAxis(3) }

    val ch5Analog = KAxis { joystick.getRawAxis(4) }
    val ch6Analog = KAxis { joystick.getRawAxis(5) }
    val ch7Analog = KAxis { joystick.getRawAxis(6) }
    val ch8Analog = KAxis { joystick.getRawAxis(7) }

    val ch5SwitchUp = KAxis(joystick, 4).activateAt(0.6 )
    val ch6SwitchUp = KAxis(joystick, 5).activateAt(0.6 )
    val ch7SwitchUp = KAxis(joystick, 6).activateAt(0.6 )
    val ch8SwitchUp = KAxis(joystick, 7).activateAt(0.6 )

    val ch5SwitchDown = KAxis(joystick, 4).activateBefore(0.4 )
    val ch6SwitchDown = KAxis(joystick, 5).activateBefore(0.4 )
    val ch7SwitchDown = KAxis(joystick, 6).activateBefore(0.4 )
    val ch8SwitchDown = KAxis(joystick, 7).activateBefore(0.4 )

    val ch5SwitchNeutral = KAxis(joystick, 4).activateBetween(0.4, 0.6 )
    val ch6SwitchNeutral = KAxis(joystick, 5).activateBetween(0.4, 0.6 )
    val ch7SwitchNeutral = KAxis(joystick, 6).activateBetween(0.4, 0.6 )
    val ch8SwitchNeutral = KAxis(joystick, 7).activateBetween(0.4, 0.6 )
}

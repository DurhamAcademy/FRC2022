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

    val ch5SwitchUp = ch5Analog.activateAt(0.6 )
    val ch6SwitchUp = ch6Analog.activateAt(0.6 )
    val ch7SwitchUp = ch7Analog.activateAt(0.6 )
    val ch8SwitchUp = ch8Analog.activateAt(0.6 )

    val ch5SwitchDown = ch5Analog.activateBefore(0.4 )
    val ch6SwitchDown = ch6Analog.activateBefore(0.4 )
    val ch7SwitchDown = ch7Analog.activateBefore(0.4 )
    val ch8SwitchDown = ch8Analog.activateBefore(0.4 )

    val ch5SwitchNeutral = ch5Analog.activateBetween(0.4, 0.6 )
    val ch6SwitchNeutral = ch6Analog.activateBetween(0.4, 0.6 )
    val ch7SwitchNeutral = ch7Analog.activateBetween(0.4, 0.6 )
    val ch8SwitchNeutral = ch8Analog.activateBetween(0.4, 0.6 )
}

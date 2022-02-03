package kyberlib.input.controller

import kyberlib.input.AxisButton
import kyberlib.input.KAxis
import kyberlib.input.KController

// todo: label these better
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

    val ch5SwitchUp = AxisButton(joystick, 4) { value: Double -> value > 0.6 }
    val ch6SwitchUp = AxisButton(joystick, 5) { value: Double -> value > 0.6 }
    val ch7SwitchUp = AxisButton(joystick, 6) { value: Double -> value > 0.6 }
    val ch8SwitchUp = AxisButton(joystick, 7) { value: Double -> value > 0.6 }

    val ch5SwitchDown = AxisButton(joystick, 4) { value: Double -> value < 0.4 }
    val ch6SwitchDown = AxisButton(joystick, 5) { value: Double -> value < 0.4 }
    val ch7SwitchDown = AxisButton(joystick, 6) { value: Double -> value < 0.4 }
    val ch8SwitchDown = AxisButton(joystick, 7) { value: Double -> value < 0.4 }

    val ch5SwitchNeutral = AxisButton(joystick, 4) { value: Double -> value in 0.4..0.6 }
    val ch6SwitchNeutral = AxisButton(joystick, 5) { value: Double -> value in 0.4..0.6 }
    val ch7SwitchNeutral = AxisButton(joystick, 6) { value: Double -> value in 0.4..0.6 }
    val ch8SwitchNeutral = AxisButton(joystick, 7) { value: Double -> value in 0.4..0.6 }
}

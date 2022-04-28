package frc.kyberlib.input.controller

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.kyberlib.command.Debug
import frc.kyberlib.input.KAxis
import frc.kyberlib.input.KButton
import frc.kyberlib.input.KController

/**
 * Represents a standard Xbox Controller
 */
class KXboxController(port: Int) : KController(port), Debug {
    val triggerSensitivity = 0.2

    val leftX = KAxis { joystick.getRawAxis(0) }
    val leftY = KAxis { joystick.getRawAxis(1) }

    val rightX = KAxis { joystick.getRawAxis(4) }
    val rightY = KAxis { joystick.getRawAxis(5) }

    val a = KButton(joystick, 1)
    val b = KButton(joystick, 2)
    val x = KButton(joystick, 3)
    val y = KButton(joystick, 4)

    val leftBumper = KButton(joystick, 5)  // these might be the menu buttons
    val rightBumper = KButton(joystick, 6)

    val leftTrigger = KAxis { joystick.getRawAxis(2) }
    val rightTrigger = KAxis { joystick.getRawAxis(3) }

    val pressedLeftStick = KButton(joystick, 9)
    val pressedRightStick = KButton(joystick, 10)

    val leftMenu = KButton(joystick, 7)
    val rightMenu = KButton(joystick, 8)

    private val DPad
        get() = joystick.pov  // up = 0, 45º increments clockwise, none = -1

    val rightDPad = KButton { DPad in 1..179 }  // 90
    val leftDPad = KButton { DPad > 180 }  // 270
    val upDPad = KButton { DPad != -1 && (DPad < 90 || DPad > 270) }  // 0
    val downDPad = KButton { DPad in 91..269 }  // 180
}

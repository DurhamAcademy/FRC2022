package kyberlib.input.controller

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import kyberlib.input.KAxis
import kyberlib.input.KController
import edu.wpi.first.wpilibj2.command.button.Trigger
import kyberlib.command.Debug

/**
 * Represents a standard Xbox Controller
 */
class KXboxController(port: Int) : KController(port), Debug {
    val triggerSensitivity = 0.2

    val leftX = KAxis { joystick.getRawAxis(0) }
    val leftY = KAxis { joystick.getRawAxis(1) }

    val rightX = KAxis{ joystick.getRawAxis(4) }
    val rightY = KAxis{ joystick.getRawAxis(5) }

    val aButton = JoystickButton(joystick, 1)
    val bButton = JoystickButton(joystick, 2)
    val xButton = JoystickButton(joystick, 3)
    val yButton = JoystickButton(joystick, 4)

    val leftBumper = JoystickButton(joystick, 5)  // these might be the menu buttons
    val rightBumper = JoystickButton(joystick, 6)

    val leftTrigger = KAxis { joystick.getRawAxis(2) }
    val rightTrigger = KAxis { joystick.getRawAxis(3) }

    private val DPad
        get() = joystick.pov  // up = 0, 45º increments clockwise, none = -1

    val rightDPad = Trigger { DPad in 1..179 }  // 90
    val leftDPad = Trigger { DPad > 180 }  // 270
    val upDPad = Trigger { DPad != 0 && (DPad < 90 || DPad > 270) }  // 0
    val downDPad = Trigger { DPad in 91..269 }  // 180

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

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "leftX" to leftX,
            "leftY" to leftY,

            "rightX" to rightX,
            "rightY" to rightY,

            "left trigger" to leftTrigger,
            "right trigger" to rightTrigger,

            "left bumper" to leftBumper.get(),
            "right bumper" to rightBumper.get(),

            "A" to aButton.get(),
            "B" to bButton.get(),
            "X" to xButton.get(),
            "Y" to yButton.get(),

            "DPAD" to DPad,
            "rumble" to rumble
        )
    }
}

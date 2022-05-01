package frc.kyberlib.input.controller

import frc.kyberlib.input.KAxis
import frc.kyberlib.input.KButton
import frc.kyberlib.input.KController

class KPlayStation(port: Int) : KController(port) {
    val square = KButton(joystick, 1)
    val cross = KButton(joystick, 2)
    val circle = KButton(joystick, 3)
    val triangle = KButton(joystick, 4)
    val leftBumber = KButton(joystick, 5)
    val rightBumper = KButton(joystick, 6)
//    val leftTrigger = KButton(joystick, 7)
//    val rightTrigger = KButton(joystick, 8)
    val share = KButton(joystick, 9)
    val options = KButton(joystick, 10)
    val pressLeftStick = KButton(joystick, 11)
    val pressRightStick = KButton(joystick, 12)
    val PS = KButton(joystick, 13)
    val touchpad = KButton(joystick, 14)

    val leftX = KAxis(joystick, 0)
    val leftY = KAxis(joystick, 1)
    val rightX = KAxis(joystick, 2)
    val rightY = KAxis(joystick, 5)
    val leftTrigger = KAxis(joystick, 3)
    val rightTrigger = KAxis(joystick, 4)

    private val DPad
        get() = joystick.pov  // up = 0, 45º increments clockwise, none = -1

    val rightDPad = KButton { DPad in 1..179 }  // 90
    val leftDPad = KButton { DPad > 180 }  // 270
    val upDPad = KButton { DPad != -1 && (DPad < 90 || DPad > 270) }  // 0
    val downDPad = KButton { DPad in 91..269 }  // 180

}
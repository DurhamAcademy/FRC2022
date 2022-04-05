package frc.robot.controls

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.kyberlib.math.invertIf
import frc.robot.RobotContainer
import kotlin.math.PI

object RocketLeague : ControlSchema2022() {
    private val xbox = RobotContainer.controller.apply {
        leftX.apply {
            maxVal = -PI * 3
            expo = 73.0
            deadband = 0.1
        }

        // throttle
        leftTrigger.apply {
            maxVal = -12.0
            expo = 20.0
            deadband = 0.2
        }
        rightTrigger.apply {
            maxVal = 12.0//12.0
            expo = 20.0
            deadband = 0.2
        }
    }

    override val DRIVE_FORWARD: Double
        get() = xbox.leftTrigger.value + xbox.rightTrigger.value
    override val DRIVE_TURN: Double
        get() = xbox.leftX.value.invertIf { DRIVE_FORWARD < 0.0 }
    override val INTAKE: Trigger = xbox.aButton
    override val SHOOT: Trigger = xbox.bButton
    override val FORCE_SHOT: Trigger = xbox.yButton
    override val EJECT: Trigger = xbox.leftMenu
    override val FLUSH: Trigger = xbox.rightMenu
    override val LOCK_TURRET: Trigger = xbox.downDPad
    override val ZERO_TURRET: Trigger = xbox.upDPad
    override val DISPOSE: Trigger = xbox.xButton

    override val CLIMB_MODE: Trigger = xbox.pressedLeftStick.and(xbox.pressedRightStick)
    override val EMOTE: Trigger = xbox.leftDPad
}
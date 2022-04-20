package frc.robot.controls

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.kyberlib.input.KButton
import frc.kyberlib.input.k
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
            maxVal = -12.0//-12.0
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
    override val INTAKE: KButton = xbox.a
    override val SHOOT: KButton = xbox.b
    override val FORCE_SHOT: KButton = xbox.y
    override val EJECT: KButton = xbox.leftMenu
    override val FLUSH: KButton = xbox.rightMenu
    override val LOCK_TURRET: KButton = xbox.downDPad
    override val ZERO_TURRET: KButton = xbox.upDPad
    override val DISPOSE: KButton = xbox.x

    override val CLIMB_MODE: KButton = xbox.pressedLeftStick.and(xbox.pressedRightStick).k
    override val EMOTE: KButton = xbox.leftDPad
}
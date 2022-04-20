package frc.robot.controls

import edu.wpi.first.wpilibj2.command.button.Button
import frc.kyberlib.input.KButton
import frc.robot.RobotContainer
import kotlin.math.PI

object DefaultControls : ControlSchema2022() {
    private val xbox = RobotContainer.controller.apply {
        rightX.apply {
            maxVal = -PI * 3.0
            expo = 100.0
            deadband = 0.1
        }

        // throttle
        leftY.apply {
            maxVal = -12.0
            expo = 20.0
            deadband = 0.2
        }
    }

    override val DRIVE_FORWARD: Double
        get() = xbox.leftY.value
    override val DRIVE_TURN: Double
        get() = xbox.rightX.value
    override val INTAKE: KButton = xbox.leftBumper
    override val SHOOT: Button = xbox.rightTrigger.activateAt(0.1)
    override val FORCE_SHOT: KButton = xbox.rightBumper
    override val EJECT: KButton = xbox.leftDPad
    override val FLUSH: KButton = xbox.rightDPad
    override val LOCK_TURRET: KButton = xbox.y
    override val ZERO_TURRET: KButton = xbox.b
    override val CLIMB_MODE: KButton = xbox.rightMenu
    override val EMOTE: KButton = xbox.upDPad
    override val DISPOSE: KButton = xbox.x
}
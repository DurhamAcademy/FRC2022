package frc.robot.controls

import edu.wpi.first.wpilibj2.command.button.Trigger
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
    override val INTAKE: Trigger = xbox.leftBumper
    override val SHOOT: Trigger = xbox.rightTrigger.activateAt(0.5)
    override val FORCE_SHOT: Trigger = xbox.rightBumper
    override val EJECT: Trigger = xbox.leftDPad
    override val FLUSH: Trigger = xbox.rightDPad
    override val LOCK_TURRET: Trigger = xbox.leftMenu
    override val CLIMB_MODE: Trigger = xbox.rightMenu
    override val EMOTE: Trigger = xbox.aButton
}
package frc.robot.controls

import edu.wpi.first.wpilibj2.command.button.Button

abstract class ControlSchema2022 {
    abstract val DRIVE_FORWARD: Double
    abstract val DRIVE_TURN: Double

    abstract val INTAKE: Button

    abstract val SHOOT: Button
    abstract val FORCE_SHOT: Button
    abstract val DISPOSE: Button

    abstract val EJECT: Button
    abstract val FLUSH: Button

    abstract val LOCK_TURRET: Button
    abstract val ZERO_TURRET: Button
    abstract val CLIMB_MODE: Button

    abstract val EMOTE: Button
}
package frc.robot.controls

import edu.wpi.first.wpilibj2.command.button.Trigger

abstract class ControlSchema2022 {
    abstract val DRIVE_FORWARD: Double
    abstract val DRIVE_TURN: Double

    abstract val INTAKE: Trigger

    abstract val SHOOT: Trigger
    abstract val FORCE_SHOT: Trigger
    abstract val DISPOSE: Trigger

    abstract val EJECT: Trigger
    abstract val FLUSH: Trigger

    abstract val LOCK_TURRET: Trigger
    abstract val ZERO_TURRET: Trigger
    abstract val CLIMB_MODE: Trigger

    abstract val EMOTE: Trigger
}
package frc.robot.controls

import frc.kyberlib.input.KButton

abstract class ControlSchema2022 {
    abstract val FORWARD: Double
    abstract val STRAFE: Double
    abstract val TURN: Double

    abstract val INTAKE: KButton

    abstract val SHOOT: KButton
    abstract val FORCE_SHOT: KButton
    abstract val DISPOSE: KButton

    abstract val EJECT: KButton
    abstract val FLUSH: KButton

    abstract val LOCK_TURRET: KButton
    abstract val ZERO_TURRET: KButton
    abstract val CLIMB_MODE: KButton

    abstract val EMOTE: KButton
}
package frc.robot.subsystems

import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.command.DebugLevel
import frc.kyberlib.command.KSubsystem
import frc.kyberlib.math.units.extensions.rotationsPerSecond
import frc.kyberlib.motorcontrol.BrushType
import frc.kyberlib.motorcontrol.KSimulatedESC
import frc.robot.commands.intake.Idle

/**
 * State of the hopper. Used for LEDs and other dependencies
 */
public enum class CONVEYOR_STATUS {
    EMPTY, SINGLE_GOOD, FULL_GOOD, BAD, FEEDING, IDLE
}

/**
 * Controls all aspects of the hopper.
 * Waiting for design to be finalized before code is added
 */
object Conveyor : KSubsystem() {
    override val priority: DebugLevel = DebugLevel.LowPriority

    val conveyor = KSparkMax(21, BrushType.BRUSHLESS).apply {
        identifier = "conveyor"
        reversed = true
        currentLimit = 20 // @Everett: this motor has a current limit of 20 because it goes into the smaller breaker
        gearRatio = 1/5.0
    }

    // @Everett: this is probably deprecated since we dont have color sensors
    var status = CONVEYOR_STATUS.FULL_GOOD // FIXME: NO! bad named variable

    val feeder = KSparkMax(22).apply {
        identifier = "feeder"
        gearRatio = 1/5.0
        currentLimit = 20
    }

    // @Everett: this is probably deprecated since we dont have color sensors
    // find usages and remove them
    val good // FIXME: NO! why pelase stop
        get() = true

    fun feed() {
        status = CONVEYOR_STATUS.FEEDING
        conveyor.velocity = 6.rotationsPerSecond
    }

    init {
        defaultCommand = Idle
    }

    override fun periodic() {
        debugDashboard()
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "indexer" to conveyor,
            "feeder" to feeder,
            "status" to status.name
        )
    }
}
package frc.robot.subsystems

import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.command.DebugLevel
import frc.kyberlib.command.KSubsystem
import frc.kyberlib.math.units.extensions.rotationsPerSecond
import frc.kyberlib.motorcontrol.BrushType
import frc.robot.commands.intake.Idle


/**
 * Controls all aspects of the hopper.
 * Waiting for design to be finalized before code is added
 */
object Conveyor : KSubsystem() {
    override val priority: DebugLevel = DebugLevel.LowPriority

    val conveyor = KSparkMax(21, BrushType.BRUSHLESS).apply {
        identifier = "conveyor"
        reversed = true
        currentLimit = 20
        gearRatio = 1/5.0
    }

    var status = ConveyorStatus.IDLE

    val feeder = KSparkMax(22).apply {
        identifier = "feeder"
        gearRatio = 1/5.0
        currentLimit = 20
    }

    @Suppress("unused")
    fun off() {
        status = CONVEYOR_STATUS.OFF
        conveyor.velocity = 0.rpm
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
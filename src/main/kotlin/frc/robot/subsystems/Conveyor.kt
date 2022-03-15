package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.motorcontrol.KSimulatedESC
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.robot.commands.intake.Idle


/**
 * State of the hopper. Used for LEDs and other dependencies
 */
enum class ConveyorStatus {
    EMPTY, SINGLE_GOOD, FULL_GOOD, BAD, FEEDING, IDLE, OFF
}
/**
 * Controls all aspects of the hopper.
 * Waiting for design to be finalized before code is added
 */
object Conveyor : SubsystemBase(), Debug {
    override val priority: DebugFilter = DebugFilter.Low

    val conveyor = KSparkMax(21).apply {
        identifier = "conveyor"
        reversed = true
        currentLimit = 20
        gearRatio = 1/5.0
    }

    var status = ConveyorStatus.IDLE

    val feeder = KSparkMax(30).apply {
        identifier = "feeder"
        gearRatio = 1/5.0
        currentLimit = 20
    }

    fun feed() {
        status = ConveyorStatus.FEEDING
        feeder.percent = 0.9
        conveyor.percent = 0.8
    }
    fun idle() {
        status = ConveyorStatus.IDLE
        feeder.percent = -0.1
        conveyor.percent = -0.0
    }
    fun stop() {
        status = ConveyorStatus.OFF
        conveyor.stop()
        feeder.stop()
    }
    init {
        defaultCommand = Idle
    }

    override fun periodic() {
//        debugDashboard()
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "indexer" to conveyor,
            "feeder" to feeder,
            "status" to status.name
        )
    }
}
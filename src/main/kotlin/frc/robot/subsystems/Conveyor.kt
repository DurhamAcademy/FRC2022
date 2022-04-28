package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.robot.commands.conveyor.Idle
import frc.robot.commands.intake.Flush

/**
 * Controls all aspects of the hopper.
 * Waiting for design to be finalized before code is added
 */
object Conveyor : SubsystemBase(), Debug {
    override val priority: DebugFilter = DebugFilter.Low

    private val conveyor = KSparkMax(21).apply {
        identifier = "conveyor"
        reversed = true
        currentLimit = 20
        gearRatio = 1 / 5.0
    }

    private val feeder = KSparkMax(30).apply {
        identifier = "feeder"
        gearRatio = 1 / 5.0
        currentLimit = 20
    }

    fun feed() {
        feeder.percent = 0.8
        conveyor.percent = 0.6
    }

    fun prepare() {
        conveyor.percent = -.05
        feeder.percent = -0.5
    }

    fun idle() {
        feeder.percent = -0.1
        conveyor.percent = -0.0
    }

    fun stop() {
        conveyor.stop()
        feeder.stop()
    }

    fun autoManage() {
        conveyor.percent = 0.1
        feeder.percent = -0.1
    }

    fun flush() {
        conveyor.percent = -1.0
    }

    fun custom(conveyorPercent: Double = 0.0, feederPercent: Double = 0.0) {
        conveyor.percent = conveyorPercent
        feeder.percent = feederPercent
    }

    init {
        defaultCommand = Idle
    }
}
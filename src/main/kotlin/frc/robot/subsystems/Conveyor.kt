package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.robot.commands.conveyor.Idle

/**
 * Controls all aspects of the hopper.
 * Waiting for design to be finalized before code is added
 */
object Conveyor : SubsystemBase() {  // todo
    val feeder = KSparkMax(30).apply {
        identifier = "feeder"
        gearRatio = 1 / 5.0
        currentLimit = 20
    }

    fun feed() {
        feeder.percent = 0.8
    }

    fun prepare() {
        feeder.percent = -0.0
    }

    fun idle() {
        feeder.percent = -0.0
    }

    fun stop() {
        feeder.stop()
    }

    init {
        defaultCommand = Idle
    }
}
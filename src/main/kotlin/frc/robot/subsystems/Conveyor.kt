package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.rotationsPerSecond
import frc.kyberlib.math.units.extensions.rpm
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
object Conveyor : SubsystemBase(), Debug {
    init {
        log("init")
    }

    val ConveyorMotor = KSparkMax(21, BrushType.BRUSHLESS).apply {
        identifier = "conveyor"
        reversed = true
        currentLimit = 20 // @Everett: this motor has a current limit of 20 because it goes into the smaller breaker
        gearRatio = 1/5.0
    }

    // @Everett: this is probably deprecated since we dont have color sensors
    var status = CONVEYOR_STATUS.FULL_GOOD // FIXME: NO! bad named variable

    val feeder = KSparkMax(22).apply {
        identifier = "feeder"
    }

    // @Everett: this is probably deprecated since we dont have color sensors
    // find usages and remove them
    val good // FIXME: NO! why pelase stop
        get() = true

    fun feed() {
        status = CONVEYOR_STATUS.FEEDING
        ConveyorMotor.velocity = 6.rotationsPerSecond
    }

    // @Everett - move this into a default command
    fun idle() {
        status = CONVEYOR_STATUS.IDLE
        ConveyorMotor.velocity = 1.rotationsPerSecond
        ConveyorMotor.maxAcceleration = 0.1.rotationsPerSecond
        // consider having the feeder motor idle backwards to prevent balls getting up ot the flywheel
    }

    init {
        defaultCommand = Idle
    }

    override fun periodic() {
        debugDashboard()
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "indexer" to ConveyorMotor,
            "feeder" to feeder,
            "status" to status.name
        )
    }
}
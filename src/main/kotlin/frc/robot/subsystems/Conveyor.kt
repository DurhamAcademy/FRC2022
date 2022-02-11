package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.command.Debug

/**
 * State of the hopper. Used for LEDs and other dependencies
 */
public enum class CONVEYOR_STATUS {
    EMPTY, SINGLE_GOOD, FULL_GOOD, BAD, FEEDING
}

/**
 * Controls all aspects of the hopper.
 * Waiting for design to be finalized before code is added
 */
object Conveyor : SubsystemBase(), Debug {
    var status = CONVEYOR_STATUS.FULL_GOOD
    
    val indexer = KSparkMax(0)
    val feeder = KSparkMax(0)

    init {
    }

    val good
        get() = true

    override fun periodic() {
        debugDashboard()
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "indexer" to indexer, 
            "feeder" to feeder,
            "status" to status.name
        )
    }
}
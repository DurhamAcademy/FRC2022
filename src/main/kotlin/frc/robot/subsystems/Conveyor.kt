package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.command.Debug
import frc.kyberlib.motorcontrol.KSimulatedESC

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
    init {
        log("init")
    }
    var status = CONVEYOR_STATUS.FULL_GOOD
    
    val indexer = KSimulatedESC(21).apply {  }
    val feeder = KSimulatedESC(22)

    val good
        get() = true

    fun feed() {
        status = CONVEYOR_STATUS.FEEDING
    }

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
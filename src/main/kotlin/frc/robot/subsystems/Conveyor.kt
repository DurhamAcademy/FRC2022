package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.rotationsPerSecond
import frc.kyberlib.math.units.extensions.rpm
import frc.kyberlib.motorcontrol.KSimulatedESC
import frc.kyberlib.motorcontrol.MotorType

/**
 * Controls all aspects of the hopper.
 * Waiting for design to be finalized before code is added
 */
object Conveyor : SubsystemBase(), Debug {
    init {
        println("Conveyor")
    }

    //FIXME: - change id
    val ConveyorMotor = KSparkMax(10, MotorType.BRUSHLESS).apply {
        identifier = "conveyor"
        reversed = false
        currentLimit = 40 //FIXME: - is this correct?
        gearRatio = 1/5.0
    }
    var status = CONVEYOR_STATUS.FULL_GOOD // FIXME: NO! bad named variable

    val indexer = KSimulatedESC(21)
    val feeder = KSimulatedESC(22)

    val good // FIXME: NO! why pelase stop
        get() = true

    fun feed() {
        status = CONVEYOR_STATUS.FEEDING
        ConveyorMotor.velocity = 6.rotationsPerSecond
    }

    fun idle() {
        status = CONVEYOR_STATUS.IDLE
        ConveyorMotor.velocity = 1.rotationsPerSecond
        ConveyorMotor.maxAcceleration = 0.1.rotationsPerSecond
    }

    @Suppress("unused")
    fun off() {
        status = CONVEYOR_STATUS.OFF
        ConveyorMotor.brakeMode= true
        ConveyorMotor.velocity = 0.rpm
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
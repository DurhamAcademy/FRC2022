package frc.kyberlib.mechanisms

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.KMotorController

/**
 * Simple pre-made system that will control an elevator. Control by setting the position.
 * @param motors a series of motors to move the elevator. Subsequent motors will follow the first
 * @param radius the radius of the wheel that moves the elevator. Alternatively rotationToVelocityConversionFactor/2π
 * @param initialPosition optional value of where the elevator starts
 */
class Elevator(leadMotor: KMotorController, initialPosition: Length = 0.feet, mass: Double) : SubsystemBase(),
    Debug {
    private val motor = leadMotor.apply {
        resetPosition(initialPosition)
        val system = elevatorSystem(mass)
        setupSim(system)
        stateSpaceControl(
            system, 2.inches, 2.inches, .5.metersPerSecond
        )
    }

    /**
     * The linear Position of where the elevator should be
     */
    var position: Length
        get() = motor.linearPosition
        set(value) {
            motor.linearPosition = value
        }

    fun update() {
        motor.updateVoltage()
    }

    fun stop() {
        motor.stop()
    }

    override fun debugValues(): Map<String, Any?> {
        return motor.debugValues()
    }
}
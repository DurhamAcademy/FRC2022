package frc.kyberlib.mechanisms

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.KMotorController

/**
 * Simple pre-made system that will control an elevator. Control by setting the angle.
 * @param leadMotor the motor driving the elevator. Other motors in the mechanism should follow this
 * @param initialPosition optional value of where the elevator starts
 * @param mass how heavy (in kg) the system is lifting
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
        get() = motor.distance
        set(value) {
            motor.distance = value
        }

    fun update() {
        motor.updateVoltage()
    }

    fun stop() {
        motor.stop()
    }
}
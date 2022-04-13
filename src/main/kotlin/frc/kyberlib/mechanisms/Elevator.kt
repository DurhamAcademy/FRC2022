package frc.kyberlib.mechanisms

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.Length
import frc.kyberlib.math.units.extensions.feet
import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.math.units.extensions.metersPerSecond
import frc.kyberlib.motorcontrol.KMotorController

/**
 * Simple pre-made system that will control an elevator. Control by setting the position.
 * @param motors a series of motors to move the elevator. Subsequent motors will follow the first
 * @param radius the radius of the wheel that moves the elevator. Alternatively rotationToVelocityConversionFactor/2π
 * @param initialPosition optional value of where the elevator starts
 */
class Elevator(leadMotor: KMotorController, initialPosition: Length = 0.feet, mass: Double) : SubsystemBase(),
    Debug {
    private val master = leadMotor.apply {
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
        get() = master.linearPosition
        set(value) {
            master.linearPosition = value
        }

    override fun debugValues(): Map<String, Any?> {
        return master.debugValues()
    }
}
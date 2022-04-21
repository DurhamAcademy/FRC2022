package frc.kyberlib.mechanisms

import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.radiansPerSecond
import frc.kyberlib.math.units.extensions.rpm
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.motorcontrol.rev.KSparkMax

class Arm(motor: KMotorController, moi: Double) {
    private val motor = motor.apply {
        brakeMode = true
        val system = armSystem(moi)
        stateSpaceControl(system, 2.degrees, 3.degrees, 1.radiansPerSecond)
        setupSim(system)
    }

    var angle
        get() = motor.position
        set(value) {
            motor.position = value
        }


    fun update() {
        motor.updateVoltage()
    }

    fun stop() {
        motor.stop()
    }
}
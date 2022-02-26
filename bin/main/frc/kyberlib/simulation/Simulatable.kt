package frc.kyberlib.simulation

import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.Time


/**
 * Interface that allows a subsystem to hook into the simulation
 */
interface Simulatable {
    /**
     * Periodic update from the Simulation
     * @param dt how long since the last update
     */
    fun simUpdate(dt: Time)

    val real: Boolean
        get() = Game.real
}
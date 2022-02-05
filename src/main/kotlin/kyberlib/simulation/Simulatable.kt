package kyberlib.simulation

import kyberlib.command.Game


/**
 * Interface that allows a subsystem to hook into the simulation
 */
interface Simulatable {
    /**
     * Periodic update from the Simulation
     * @param dt how long since the last update
     */
    fun simUpdate(dt: Double)

    val real: Boolean
        get() = Game.real
}
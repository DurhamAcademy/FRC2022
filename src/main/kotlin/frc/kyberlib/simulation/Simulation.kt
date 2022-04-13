package frc.kyberlib.simulation

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.Time
import frc.kyberlib.simulation.field.KField2d

/**
 * Simulation that will run a loop to update simulatable objects
 */
object Simulation : SubsystemBase() {
    // stores things to be sim updated
    private val sims = mutableSetOf<Simulatable>()

    // stores time values
    private var prevTime = Game.time
    private val time: Time
        get() = Game.time
    private val startTime = time
    val elapsedTime
        get() = time - startTime
    val dt
        get() = time - prevTime

    // field to draw robot
    val field = KField2d
    init {
        SmartDashboard.putData("Field", field)
    }

    /**
     * Update all attached simulations
     */
    override fun simulationPeriodic() {
        val dt = time - prevTime
        for (sim in sims) {
            sim.simUpdate(dt)
        }
        prevTime = time
    }

    /**
     * Add object to be periodically updated during simulation
     */
    fun include(simulatable: Simulatable) {
        sims.add(simulatable)
    }
}
package frc.kyberlib.simulation

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.Time
import frc.kyberlib.math.units.extensions.seconds
import frc.kyberlib.simulation.field.KField2d
import kotlin.time.Duration.Companion.seconds

/**
 * Simulation that will run a loop to update simulatable objects
 */
class Simulation : SubsystemBase() {  // TODO: figure out the hardware tab
    companion object {
        val instance: Simulation
            get() { return if(internal == null) Simulation() else internal!! }
        private var internal: Simulation? = null
    }
    init { internal = this }

    private val sims = ArrayList<Simulatable>()

    // stores time values
    private var prevTime = Game.time
    private val time: Time
        get() = Game.time
    private val startTime = time
    val elapsedTime
        get() = time - startTime
    val dt
        get() = time - prevTime

    var chassisFF = SimpleMotorFeedforward(0.0, 0.0)

    /**
     * Takes voltage and velocity and calculates what acceleration should be
     */
    fun inverseFF(voltage: Double, velocity: Double): Double = ((voltage / chassisFF.ks) - chassisFF.kv * velocity) / chassisFF.ka

    // field to draw robot
    val field = KField2d

    init {
        assert(Game.sim) {"should not be simulating from real robot"}
        SmartDashboard.putData("Field", field)
    }

    /**
     * Update all the attached simulatables
     */
    override fun periodic() {
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
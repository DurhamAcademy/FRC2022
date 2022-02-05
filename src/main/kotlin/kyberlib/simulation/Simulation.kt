package kyberlib.simulation

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kyberlib.command.Game
import kyberlib.simulation.field.KField2d

/**
 * Simulation that will run a loop to update simulatable objects
 */
class Simulation : SubsystemBase() {
    companion object {
        val instance: Simulation
            get() { return if(internal == null) Simulation() else internal!! }
        private var internal: Simulation? = null
    }
    init { internal = this }

    private val sims = ArrayList<Simulatable>()

    // stores time values
    private var prevTime = -1.0
    private val time: Double
        get() = Game.time.toDouble()
    private val startTime = time
    val elapsedTime
        get() = time - startTime
    val dt
        get() = time - prevTime

    var chassisFF: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0)

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
        if (prevTime < 0) {
            prevTime = time
            return
        }
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
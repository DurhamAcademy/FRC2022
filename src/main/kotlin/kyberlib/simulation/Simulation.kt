package kyberlib.simulation

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kyberlib.simulation.field.KField2d

/**
 * Simulation that will run a loop to update simulatable objects
 */
class Simulation : SubsystemBase() {
    companion object {
        val instance: Simulation
            get() { return if(internal == null) Simulation() else internal!! }
        private var internal: Simulation? = null

        val real:Boolean
            get() = RobotBase.isReal()
    }
    init { internal = this }

    private val sims = ArrayList<Simulatable>()

    // stores time values
    private var prevTime = -1.0
    private val time: Double
        get() = Timer.getFPGATimestamp()
    private val startTime = time
    val elapsedTime
        get() = time - startTime
    val dt
        get() = time - prevTime

    // field to draw robot
    val field = KField2d

    init {
        assert(!real) {"should not be simulating from real robot"}
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
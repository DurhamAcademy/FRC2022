package frc.kyberlib.mechanisms

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.simulation.Simulatable

/**
 * Pre-made Flywheel Subsystem. Also a demo of StateSpace control from WPILIB. Control using velocity variable.
 */
class Flywheel(  // todo: allow for gooder constructor
    leadMotor: KMotorController,
    plant: LinearSystem<N1, N1, N1>, // kg * m^2
    timeDelay: Double = 0.02
) : Debug {

    constructor(leadMotor: KMotorController, ff: SimpleMotorFeedforward, timeDelay: Double = 0.02) : this(leadMotor, leadMotor.velocitySystem(ff), timeDelay)
    constructor(leadMotor: KMotorController, kFlywheelMomentOfInertia: Double = 0.00032, timeDelay: Double = 0.02) : this(leadMotor, leadMotor.flywheelSystem(kFlywheelMomentOfInertia), timeDelay)
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-flywheel-walkthrough.html

    private val motor = leadMotor.apply {
        stateSpaceControl(plant, 3.radiansPerSecond, 8.radiansPerSecond)
        setupSim(plant)
    }

    /**
     * The plant holds a state-space model of our flywheel. This system has the following properties:
     * States: [velocity], in radians per second.
     * Inputs (what we can "put in"): [voltage], in volts.
     * Outputs (what we can measure): [velocity], in radians per second.
     */
    private val observer: KalmanFilter<N1, N1, N1> = KalmanFilter(
        N1.instance, N1.instance,
        plant,
        VecBuilder.fill(3.0),  // How accurate we think our model is
        VecBuilder.fill(0.01),  // How accurate we think our encoder
        timeDelay
    )

    /**
     * The place for tuning the plant.
     * Increasing [q-elms] punishes wrong inputs, creating agressive compensation
     * Increasing [r-elms] to punish output error, lower is more agressive
     */
    private val optimizer = LinearQuadraticRegulator(
        plant,
        VecBuilder.fill(8.0),  // q-elms. Velocity error tolerance, in radians per second.
        VecBuilder.fill(12.0),  // r-elms. 12 cause max battery voltage
        timeDelay  // estimated loop time. 0.020 for TimedRobot, but lower if using notifiers.
    )

    /**
     * The state-space loop combines a controller, observer, feedforward and plant for easy control.
     */
    private val loop = LinearSystemLoop(plant, optimizer, observer, 12.0, timeDelay)

    var velocity: AngularVelocity
        get() = motor.velocity
        set(value) {
            motor.velocity = value
        }

    fun update() {
        motor.updateVoltage()
    }

    fun stop() {
        velocity = 0.rpm
        motor.stop()
    }
}
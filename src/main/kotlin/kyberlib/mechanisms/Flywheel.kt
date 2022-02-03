package kyberlib.mechanisms

import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator
import edu.wpi.first.wpilibj.estimator.KalmanFilter
import edu.wpi.first.wpilibj.system.LinearSystemLoop
import edu.wpi.first.wpilibj.system.plant.DCMotor
import edu.wpi.first.wpilibj.system.plant.LinearSystemId
import edu.wpi.first.wpiutil.math.VecBuilder
import edu.wpi.first.wpiutil.math.numbers.N1
import kyberlib.command.Debug
import kyberlib.math.units.extensions.AngularVelocity
import kyberlib.motorcontrol.KMotorController
import kyberlib.math.units.extensions.radiansPerSecond

/**
 * Pre-made Flywheel Subsystem. Also a demo of StateSpace control from WPILIB. Control using velocity variable.
 * @param motor the controlling motor of the flywheel. If other motors are involves, make them follow this
 */
class Flywheel(private val motor: KMotorController,
               private val kFlywheelMomentOfInertia: Double = 0.00032, // kg * m^2
               private val timeDelay: Double = 0.02
               ) : Debug {
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-flywheel-walkthrough.html

    private val kFlywheelGearing = motor.gearRatio

    /**
     * The plant holds a state-space model of our flywheel. This system has the following properties:
     * States: [velocity], in radians per second.
     * Inputs (what we can "put in"): [voltage], in volts.
     * Outputs (what we can measure): [velocity], in radians per second.
     */
    private val plant = LinearSystemId.createFlywheelSystem(
        DCMotor.getNEO(2),
        kFlywheelMomentOfInertia,
        kFlywheelGearing
    )

    private val observer: KalmanFilter<N1, N1, N1> = KalmanFilter(
        N1.instance, N1.instance,
        plant,
        VecBuilder.fill(3.0),  // How accurate we think our model is
        VecBuilder.fill(0.01),  // How accurate we think our encoder
        // data is
        0.020
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


    init {
        motor.customControl = {
            loop.nextR = VecBuilder.fill(it.velocitySetpoint.radiansPerSecond)  // r = reference (setpoint)
            loop.correct(VecBuilder.fill(it.velocity.radiansPerSecond))  // update with empirical
            loop.predict(timeDelay)  // math
            val nextVoltage = loop.getU(0)  // input
            nextVoltage
        }  // todo: this wont update frequently enough, add notifier
    }

    var velocity: AngularVelocity
        get() = motor.velocity
        set(value) {motor.velocity = value}

    override fun debugValues(): Map<String, Any?> = motor.debugValues()

}
package frc.kyberlib.motorcontrol.statespace

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.LinearSystemLoop
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.Voltage

object Statespace {
    /**
     * Creates a Kalman Filter to ensure measurement accuracy.
     *
     * A Kalman filter combines math of what should happen and noisy measurements of state to give a combined better estimates
     *
     * @param plant the system to model your motor
     * @param modelDeviation how much your math can be off
     * @param measurementDeviation how much your measurements can be off
     * @param timeDelay how long between each update
     * @return a Kalman filter combining these parameters with the same dimensions as the plant
     */
    @JvmName("velocityObserver")
    fun observer(
        plant: LinearSystem<N1, N1, N1>,
        modelDeviation: AngularVelocity,
        measurementDeviation: AngularVelocity,
        timeDelay: Time = 0.02.seconds
    ): KalmanFilter<N1, N1, N1> {
        return KalmanFilter(
            N1.instance, N1.instance,
            plant,
            VecBuilder.fill(modelDeviation.radiansPerSecond),  // How accurate we think our model is
            VecBuilder.fill(measurementDeviation.radiansPerSecond),  // How accurate we think our encoder
            timeDelay.seconds
        )

    }

    @JvmName("positionObserver")
    fun observer(
        plant: LinearSystem<N2, N1, N1>,
        modelDeviation: Angle,
        measurementDeviation: Angle,
        timeDelay: Time = 0.02.seconds
    ): KalmanFilter<N2, N1, N1> {
        return KalmanFilter(
            N2.instance, N1.instance,
            plant,
            VecBuilder.fill(modelDeviation.radians, modelDeviation.radians),  // How accurate we think our model is
            VecBuilder.fill(measurementDeviation.radians),  // How accurate we think our encoder
            timeDelay.seconds
        )

    }

    @JvmName("dualObserver")
    fun observer(
        plant: LinearSystem<N2, N1, N2>,
        modelDeviation: Angle,
        measurementDeviation: Angle,
        timeDelay: Time = 0.02.seconds
    ): KalmanFilter<N2, N1, N2> {
        return KalmanFilter(
            N2.instance, N2.instance,
            plant,
            VecBuilder.fill(modelDeviation.radians, modelDeviation.radians),  // How accurate we think our model is
            VecBuilder.fill(measurementDeviation.radians, measurementDeviation.radians),  // How accurate we think our encoder
            timeDelay.seconds
        )

    }
    /**
     * A Linear Quadractic Regulator takes a system and the cost of state error and voltage error and creates an optimal adjustment scheme.
     * @param plant system to model the motor
     * @param velocityTolerance how far of from the desired state is acceptables
     * @param voltageTolerance what is the max acceptable voltage. Default to battery voltage (12.0)
     * @return a LQR that will optimize the control system for your plant
     */

    @JvmName("velocityOptimizer")
    fun optimizer(
        plant: LinearSystem<N1, N1, N1>,
        velocityTolerance: AngularVelocity,
        voltageTolerance: Voltage = 12.0,
        timeDelay: Time = 0.02.seconds
    ): LinearQuadraticRegulator<N1, N1, N1> {
        return LinearQuadraticRegulator(
            plant,
            VecBuilder.fill(velocityTolerance.radiansPerSecond),  // q-elms. Velocity error tolerance, in radians per second.
            VecBuilder.fill(voltageTolerance),  // r-elms. 12 cause max battery voltage
            timeDelay.seconds  // estimated loop time. 0.020 for TimedRobot, but lower if using notifiers.
        )
    }

    @JvmName("positionOptimizer")
    fun optimizer(
        plant: LinearSystem<N2, N1, N1>,
        velocityTolerance: AngularVelocity,
        positionTolerance: Angle,
        voltageTolerance: Voltage = 12.0,
        timeDelay: Time = 0.02.seconds
    ): LinearQuadraticRegulator<N2, N1, N1> {
        return LinearQuadraticRegulator(
            plant,
            VecBuilder.fill(
                positionTolerance.radians,
                velocityTolerance.radiansPerSecond
            ),  // q-elms. Velocity error tolerance, in radians per second.
            VecBuilder.fill(voltageTolerance),  // r-elms. 12 cause max battery voltage
            timeDelay.seconds  // estimated loop time. 0.020 for TimedRobot, but lower if using notifiers.
        )
    }

    @JvmName("dualOptimizer")
    fun optimizer(
        plant: LinearSystem<N2, N1, N2>,
        velocityTolerance: AngularVelocity,
        positionTolerance: Angle,
        voltageTolerance: Voltage = 12.0,
        timeDelay: Time = 0.02.seconds
    ): LinearQuadraticRegulator<N2, N1, N2> {
        return LinearQuadraticRegulator(
            plant,
            VecBuilder.fill(
                positionTolerance.radians,
                velocityTolerance.radiansPerSecond
            ),  // q-elms. Velocity error tolerance, in radians per second.
            VecBuilder.fill(voltageTolerance),  // r-elms. 12 cause max battery voltage
            timeDelay.seconds  // estimated loop time. 0.020 for TimedRobot, but lower if using notifiers.
        )
    }

    /**
     * Generates a System Loop that is a combination of the system, kalman filer, and LQR
     *
     * The parameters are the same as the functions above.
     */
    @JvmName("velocitySystemLoop")
    fun systemLoop(
        plant: LinearSystem<N1, N1, N1>,
        modelDeviation: AngularVelocity, measurementDeviation: AngularVelocity,
        velocityTolerance: AngularVelocity,
        voltageTolerance: Voltage = 12.0,
        timeDelay: Time = 0.02.seconds
    ): LinearSystemLoop<N1, N1, N1> {
        val kalman = observer(plant, modelDeviation, measurementDeviation, timeDelay)
        val lqr = optimizer(plant, velocityTolerance, voltageTolerance, timeDelay)
        return LinearSystemLoop(plant, lqr, kalman, 10.0, timeDelay.seconds)
    }

    @JvmName("positionSystemLoop")
    fun systemLoop(
        plant: LinearSystem<N2, N1, N1>,
        modelDeviation: Angle, measurementDeviation: Angle,
        positionTolerance: Angle, velocityTolerance: AngularVelocity,
        inputCost: Double = 12.0,
        timeDelay: Time = 0.02.seconds
    ): LinearSystemLoop<N2, N1, N1> {
        val observer = observer(plant, modelDeviation, measurementDeviation, timeDelay)
        val optimizer = optimizer(plant, velocityTolerance, positionTolerance, inputCost, timeDelay)
        return LinearSystemLoop(plant, optimizer, observer, 10.0, timeDelay.seconds)
    }

    @JvmName("dualSystemLoop")
    fun systemLoop(
        plant: LinearSystem<N2, N1, N2>,
        modelDeviation: Angle, measurementDeviation: Angle,
        positionTolerance: Angle, velocityTolerance: AngularVelocity,
        inputCost: Double = 12.0,
        timeDelay: Time = 0.02.seconds
    ): LinearSystemLoop<N2, N1, N2> {
        val observer = observer(plant, modelDeviation, measurementDeviation, timeDelay)
        val optimizer = optimizer(plant, velocityTolerance, positionTolerance, inputCost, timeDelay)
        return LinearSystemLoop(plant, optimizer, observer, 10.0, timeDelay.seconds)
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.kyberlib.motorcontrol.statespace

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Num
import edu.wpi.first.math.StateSpaceUtil
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.UnscentedKalmanFilter
import edu.wpi.first.math.numbers.N1
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.seconds
import org.ejml.MatrixDimensionException
import org.ejml.simple.SimpleMatrix

/**
 * Combines a controller, feedforward, and observer for controlling a mechanism with full state
 * feedback.
 *
 *
 * For everything in this file, "inputs" and "outputs" are defined from the perspective of the
 * plant. This means U is an input and Y is an output (because you give the plant U (powers) and it
 * gives you back a Y (sensor values). This is the opposite of what they mean from the perspective
 * of the controller (U is an output because that's what goes to the motors and Y is an input
 * because that's what comes back from the sensors).
 *
 *
 * For more on the underlying math, read
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
 */
class LatentStateLoop<States : Num?, Inputs : Num?, Outputs : Num?>(
    /**
     * Return the controller used internally.
     *
     * @return the controller used internally.
     */
    private val controller: LinearQuadraticRegulator<States, Inputs, Outputs>,
    /**
     * Return the feedforward used internally.
     *
     * @return the feedforward used internally.
     */
    private val feedforward: LinearPlantInversionFeedforward<States, Inputs, Outputs>,
    private val observer: UnscentedKalmanFilter<States, Inputs, Outputs>,
    private val maxVoltageVolts: Double,
    private val latency: Double
)  {
    /**
     * Set the next reference r.
     *
     * @param nextR Next reference.
     */
    var nextR: Matrix<States, N1>
    /**
     * Returns the observer's state estimate x-hat.
     *
     * @return the observer's state estimate x-hat.
     */
    val xHat: Matrix<States, N1>
        get() = observer.xhat

    /**
     * Returns an element of the observer's state estimate x-hat.
     *
     * @param row Row of x-hat.
     * @return the e1-th element of the observer's state estimate x-hat.
     */
    fun getXHat(row: Int): Double {
        return observer.getXhat(row)
    }

    /**
     * Set the initial state estimate x-hat.
     *
     * @param xhat The initial state estimate x-hat.
     */
    fun setXHat(xhat: Matrix<States, N1?>?) {
        observer.xhat = xhat
    }

    /**
     * Set an element of the initial state estimate x-hat.
     *
     * @param row Row of x-hat.
     * @param value Value for element of x-hat.
     */
    fun setXHat(row: Int, value: Double) {
        observer.setXhat(row, value)
    }

    /**
     * Returns an element of the controller's next reference r.
     *
     * @param row Row of r.
     * @return the element e1 of the controller's next reference r.
     */
    fun getNextR(row: Int): Double {
        return nextR[row, 0]
    }

    /**
     * Set the next reference r.
     *
     * @param nextR Next reference.
     */
    fun setNextR(vararg nextR: Double) {
        if (nextR.size != this.nextR.numRows) {
            throw MatrixDimensionException(
                String.format(
                    "The next reference does not have the "
                            + "correct number of entries! Expected %s, but got %s.",
                    this.nextR.getNumRows(), nextR.size
                )
            )
        }
        this.nextR = Matrix(SimpleMatrix(this.nextR.getNumRows(), 1, true, nextR))
    }

    /**
     * Returns the controller's calculated control input u plus the calculated feedforward u_ff.
     *
     * @return the calculated control input u.
     */
    val u: Matrix<Inputs, N1>
        get() = clampInput(controller.u.plus(feedforward.uff))

    /**
     * Returns an element of the controller's calculated control input u.
     *
     * @param row Row of u.
     * @return the calculated control input u at the row e1.
     */
    fun getU(row: Int): Double {
        return u[row, 0]
    }

    /**
     * Zeroes reference r and controller output u. The previous reference of the
     * PlantInversionFeedforward and the initial state estimate of the KalmanFilter are set to the
     * initial state provided.
     *
     * @param initialState The initial state.
     */
    fun reset(initialState: Matrix<States, N1>?) {
        nextR.fill(0.0)
        controller.reset()
        feedforward.reset(initialState)
        observer.xhat = initialState
    }

    /**
     * Returns difference between reference r and current state x-hat.
     *
     * @return The state error matrix.
     */
    val error: Matrix<States, N1>
        get() = controller.r.minus(observer.xhat)

    /**
     * Returns difference between reference r and current state x-hat.
     *
     * @param index The index of the error matrix to return.
     * @return The error at that index.
     */
    fun getError(index: Int): Double {
        return controller.r.minus(observer.xhat)[index, 0]
    }

    val latComp = LatencyComp<States, Inputs, Outputs>()
    /**
     * Correct the state estimate x-hat using the measurements in y.
     *
     * @param y Measurement vector.
     */
    fun correct(y: Matrix<Outputs, N1?>?) {
        observer.correct(u, y)
        latComp.applyPastGlobalMeasurement(observer, 0.02, y, {u, y -> observer.correct(u, y)}, Game.time.seconds - latency)
    }

    /**
     * Sets new controller output, projects model forward, and runs observer prediction.
     *
     *
     * After calling this, the user should send the elements of u to the actuators.
     *
     * @param dtSeconds Timestep for model update.
     */
    fun predict(dtSeconds: Double) {
        val u = clampInput(
            controller
                .calculate(observer.xhat, nextR)
                .plus(feedforward.calculate(nextR))
        )
        observer.predict(u, dtSeconds)
    }

    /**
     * Clamp the input u to the min and max.
     *
     * @param unclampedU The input to clamp.
     * @return The clamped input.
     */
    private fun clampInput(unclampedU: Matrix<Inputs, N1?>?): Matrix<Inputs, N1> {
        return StateSpaceUtil.desaturateInputVector<Inputs?>(unclampedU, maxVoltageVolts)
    }

    init {
        nextR = Matrix(SimpleMatrix(controller.k.numCols, 1))
        reset(nextR)
    }
}

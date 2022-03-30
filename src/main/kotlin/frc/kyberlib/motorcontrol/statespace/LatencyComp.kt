// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.kyberlib.motorcontrol.statespace

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Num
import edu.wpi.first.math.estimator.UnscentedKalmanFilter
import edu.wpi.first.math.numbers.N1
import java.util.function.BiConsumer

class LatencyComp<S : Num?, I : Num?, O : Num?> internal constructor() {
    private val pastObserverSnapshots: MutableList<Map.Entry<Double, ObserverSnapshot>>

    /** Clears the observer snapshot buffer.  */
    fun reset() {
        pastObserverSnapshots.clear()
    }

    /**
     * Add past observer states to the observer snapshots list.
     *
     * @param observer The observer.
     * @param u The input at the timestamp.
     * @param localY The local output at the timestamp
     * @param timestampSeconds The timesnap of the state.
     */
    fun addObserverState(
        observer: UnscentedKalmanFilter<S, I, O>,
        u: Matrix<I, N1>,
        localY: Matrix<O, N1>,
        timestampSeconds: Double
    ) {
        pastObserverSnapshots.add(
            java.util.Map.entry(timestampSeconds, ObserverSnapshot(observer, u, localY))
        )
        if (pastObserverSnapshots.size > kMaxPastObserverStates) {
            pastObserverSnapshots.removeAt(0)
        }
    }

    /**
     * Add past global measurements (such as from vision)to the estimator.
     *
     * @param <R> The rows in the global measurement vector.
     * @param observer The observer to apply the past global measurement.
     * @param nominalDtSeconds The nominal timestep.
     * @param y The measurement.
     * @param globalMeasurementCorrect The function take calls correct() on the observer.
     * @param timestampSeconds The timestamp of the measurement.
    </R> */
    fun <R : Num?> applyPastGlobalMeasurement(
        observer: UnscentedKalmanFilter<S, I, O>,
        nominalDtSeconds: Double,
        y: Matrix<R, N1?>?,
        globalMeasurementCorrect: BiConsumer<Matrix<I, N1>, Matrix<R, N1?>?>,
        timestampSeconds: Double
    ) {
        if (pastObserverSnapshots.isEmpty()) {
            // State map was empty, which means that we got a past measurement right at startup. The only
            // thing we can really do is ignore the measurement.
            return
        }

        // Use a less verbose name for timestamp
        val maxIdx = pastObserverSnapshots.size - 1
        var low = 0
        var high = maxIdx

        // Perform a binary search to find the index of first snapshot whose
        // timestamp is greater than or equal to the global measurement timestamp
        while (low != high) {
            val mid = (low + high) / 2
            if (pastObserverSnapshots[mid].key < timestampSeconds) {
                // This index and everything under it are less than the requested timestamp. Therefore, we
                // can discard them.
                low = mid + 1
            } else {
                // t is at least as large as the element at this index. This means that anything after it
                // cannot be what we are looking for.
                high = mid
            }
        }
        val indexOfClosestEntry: Int = if (low == 0) {
            // If the global measurement is older than any snapshot, throw out the
            // measurement because there's no state estimate into which to incorporate
            // the measurement
            if (timestampSeconds < pastObserverSnapshots[low].key) {
                return
            }

            // If the first snapshot has same timestamp as the global measurement, use
            // that snapshot
            0
        } else if (low == maxIdx && pastObserverSnapshots[low].key < timestampSeconds) {
            // If all snapshots are older than the global measurement, use the newest
            // snapshot
            maxIdx
        } else {
            // Index of snapshot taken after the global measurement
            val nextIdx = low

            // Index of snapshot taken before the global measurement. Since we already
            // handled the case where the index points to the first snapshot, this
            // computation is guaranteed to be nonnegative.
            val prevIdx = nextIdx - 1

            // Find the snapshot closest in time to global measurement
            val prevTimeDiff = Math.abs(timestampSeconds - pastObserverSnapshots[prevIdx].key)
            val nextTimeDiff = Math.abs(timestampSeconds - pastObserverSnapshots[nextIdx].key)
            if (prevTimeDiff <= nextTimeDiff) prevIdx else nextIdx
        }
        var lastTimestamp = pastObserverSnapshots[indexOfClosestEntry].key - nominalDtSeconds

        // We will now go back in time to the state of the system at the time when
        // the measurement was captured. We will reset the observer to that state,
        // and apply correction based on the measurement. Then, we will go back
        // through all observer states until the present and apply past inputs to
        // get the present estimated state.
        for (i in indexOfClosestEntry until pastObserverSnapshots.size) {
            val key = pastObserverSnapshots[i].key
            val snapshot = pastObserverSnapshots[i].value
            if (i == indexOfClosestEntry) {
                observer.setP(snapshot.errorCovariances)
                observer.setXhat(snapshot.xHat)
            }
            observer.predict(snapshot.inputs, key - lastTimestamp)
            observer.correct(snapshot.inputs, snapshot.localMeasurements)
            if (i == indexOfClosestEntry) {
                // Note that the measurement is at a timestep close but probably not exactly equal to the
                // timestep for which we called predict.
                // This makes the assumption that the dt is small enough that the difference between the
                // measurement time and the time that the inputs were captured at is very small.
                globalMeasurementCorrect.accept(snapshot.inputs, y)
            }
            lastTimestamp = key
            pastObserverSnapshots[i] = java.util.Map.entry(
                key, ObserverSnapshot(observer, snapshot.inputs, snapshot.localMeasurements)
            )
        }
    }

    /** This class contains all the information about our observer at a given time.  */
    inner class ObserverSnapshot internal constructor(
        observer: UnscentedKalmanFilter<S, I, O>, u: Matrix<I, N1>, localY: Matrix<O, N1>
    ) {
        val xHat: Matrix<S, N1>
        val errorCovariances: Matrix<S, S>
        val inputs: Matrix<I, N1>
        val localMeasurements: Matrix<O, N1>

        init {
            xHat = observer.xhat
            errorCovariances = observer.p
            inputs = u
            localMeasurements = localY
        }
    }

    companion object {
        private const val kMaxPastObserverStates = 300
    }

    init {
        pastObserverSnapshots = ArrayList()
    }
}
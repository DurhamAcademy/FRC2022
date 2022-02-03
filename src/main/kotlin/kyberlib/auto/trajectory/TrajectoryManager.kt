package kyberlib.auto.trajectory

/**
 * Central location to store trajectories
 */
object TrajectoryManager {
    internal val trajectories = mutableMapOf<String, KTrajectory>()
    operator fun get(s: String?) = trajectories[s]
    val list
        get() = trajectories.keys.toTypedArray()
}

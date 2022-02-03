package kyberlib.auto.trajectory

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil
import kyberlib.KyberlibConfig
import kyberlib.command.Debug
import java.io.File

/**
 * Wrapper class of standard trajectory that allows for more generation options, saving / loading, and hashing
 */
class KTrajectory(private val name: String, trajectory: Trajectory, newConfig: KTrajectoryConfig?) : Trajectory(trajectory.states), Debug {
    constructor(name: String, waypoints: List<Pose2d>, config: KTrajectoryConfig? = null) : this(name, generateTrajectory(waypoints, config), config)

    constructor(name: String, startPose2d: Pose2d, waypoints: Collection<Translation2d>, config: KTrajectoryConfig? = null) : this(name, generateTrajectory(startPose2d, waypoints.toMutableList(), config), config)  // check if .toMutableList maintains order

    constructor(name: String, startPose2d: Pose2d, waypoints: Collection<Translation2d>, endPose2d: Pose2d, config: KTrajectoryConfig? = null) : this(name, generateTrajectory(startPose2d, waypoints.toMutableList(), endPose2d, config), config)

    companion object {
        var generalConfig: KTrajectoryConfig? = null

        /**
         * Load a trajectory from a file
         * @throws NoSuchFileException if it is an invalid trajectory
         */
        fun load(name: String): KTrajectory {
            val jsonFile = File("${KyberlibConfig.TRAJECTORY_PATH}/$name.json")
            val configFile = File("${KyberlibConfig.TRAJECTORY_PATH}/${name}Config.json")
            val wpiTrajectory = TrajectoryUtil.deserializeTrajectory(jsonFile.readText())
            val config = KTrajectoryConfig.load(configFile)
            return KTrajectory(name, wpiTrajectory, config)
        }

        private fun generateTrajectory(startPose2d: Pose2d, waypoints: MutableList<Translation2d>, newConfig: KTrajectoryConfig?): Trajectory {
            val config = putConfig(newConfig)
            val endpoint = waypoints.removeLast()
            val finalDelta = endpoint.minus(waypoints.last())
            val finalRotation = Rotation2d(finalDelta.x, finalDelta.y)
            return TrajectoryGenerator.generateTrajectory(
                startPose2d,
                waypoints,
                Pose2d(endpoint, finalRotation),
                config
            )
        }
        private fun generateTrajectory(startPose2d: Pose2d, waypoints: MutableList<Translation2d>, endPose2d: Pose2d, newConfig: KTrajectoryConfig?): Trajectory {
            val config = putConfig(newConfig)
            return TrajectoryGenerator.generateTrajectory(
                startPose2d,
                waypoints,
                endPose2d,
                config
            )
        }
        private fun generateTrajectory(waypoints: List<Pose2d>, newConfig: KTrajectoryConfig?): Trajectory {
            val config = putConfig(newConfig)
            return TrajectoryGenerator.generateTrajectory(waypoints, config)
        }

        /**
         * Checks whether to used the proposed Trajectory or the static one.
         * Allows for leaving out config if static has been set.
         * @param config nullable config value
         * @return a valid config for trajectory generation
         * @throws AssertionError if config is null and static is not set. Either set class config or provide one
         */
        private fun putConfig(config: KTrajectoryConfig?): KTrajectoryConfig {
            if (config == null) {
                assert(generalConfig != null) {"You must either provide a config or initialize the KTrajectory.config"}
                return generalConfig!!
            }
            return config
        }
    }

    /**
     * Save the trajectory to a file
     * @param debug print the process
     */
    fun save(debug: Boolean = false) {
        val trajFolder = File(KyberlibConfig.TRAJECTORY_PATH)
        if (!trajFolder.exists()) {
            println("Main trajectory directory does not exist, creating...")
            trajFolder.mkdir()
        }

        val jsonFile = File("${KyberlibConfig.TRAJECTORY_PATH}/$name.json")
        val configFile = File("${KyberlibConfig.TRAJECTORY_PATH}/${name}Config.json")
        val hashFile = File("${KyberlibConfig.TRAJECTORY_PATH}/$name.hash")
        if (jsonFile.exists() && hashFile.exists()) {
            if (hash != hashFile.readText().toInt()) {
                if (debug) println("Trajectory $name out of date, recreating...")
            } else {
                if (debug) println("already saved")
                return
            }
        } else {
            if (debug) println("Trajectory $name does not exist, generating...")
        }

        jsonFile.writeText(TrajectoryUtil.serializeTrajectory(this))
        hashFile.writeText(hash.toString())
        config.save(configFile)
    }

    init {
        // add to Map of trajectories
        TrajectoryManager.trajectories[name] = this
    }

    /**
     * The config that the trajectory was generated with
     */
    private val config = putConfig(newConfig)

    private val hash
        get() = hashCode()

    /**
     * Creates a hash to determine uniqueness of trajectories
     */
    override fun hashCode(): Int {
        return "${states.hashCode()},${config.hashCode()}".hashCode()
    }

    /**
     * Checks if two trajectories are identical
     */
    override fun equals(other: Any?): Boolean {
        return hash == other.hashCode()
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "start Pose" to states.first().poseMeters,
            "end Pose" to states.last().poseMeters,
            "time" to states.last().timeSeconds
        )
    }
}

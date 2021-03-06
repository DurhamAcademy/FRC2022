package frc.kyberlib.auto.trajectory

import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrajectoryUtil
import edu.wpi.first.wpilibj.Filesystem

/**
 * Central location to store trajectories
 */
object TrajectoryManager {
    val TRAJECTORY_PATH = Filesystem.getDeployDirectory().toPath().resolve("PathWeaver/output")
    val AUTO_PATH = Filesystem.getDeployDirectory().toPath().resolve("PathWeaver/Autos")
    val trajectories = mutableMapOf<String, KTrajectory>()

    fun load() {
        for (file in paths) trajectories[file] = KTrajectory(file, getPath(file))
    }

    fun release() {
        trajectories.clear()
    }

    private fun getAuto(name: String): Trajectory {
        val file = AUTO_PATH.resolve(name).toFile()

        var traj = Trajectory()
        file.readLines().forEach {
            println(it)
            val path: Trajectory =
                if (it in paths) getPath(it) else if (it in routines) getAuto(it) else throw UnknownError("Routine not found in deploy directory")
            traj = traj.concatenate(path)
        }
        return traj
    }

    private fun getPath(name: String): Trajectory =
        TrajectoryUtil.fromPathweaverJson(TRAJECTORY_PATH.resolve("$name.wpilib.json"))

    operator fun get(s: String) = trajectories[s]
    val routines = AUTO_PATH.toFile().list()!!
    val paths = TRAJECTORY_PATH.toFile().list()!!.map { it.removeSuffix(".wpilib.json") }
}

// ftp://roborio-TEAM-frc.local/home/lvuser/deploy/
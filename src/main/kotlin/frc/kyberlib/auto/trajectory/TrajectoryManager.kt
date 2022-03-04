package frc.kyberlib.auto.trajectory

import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrajectoryUtil
import edu.wpi.first.wpilibj.Filesystem
import java.io.File

/**
 * Central location to store trajectories
 */
object TrajectoryManager {
    var TRAJECTORY_PATH: String = Filesystem.getDeployDirectory().path + "/Pathweaver/output"
    var AUTO_PATH = Filesystem.getDeployDirectory().path + "/Pathweaver/Autos"

    fun getAuto(name: String): Trajectory {
        val file =  File("$AUTO_PATH/$name")

        var traj = Trajectory()
        file.readLines().forEach {
            val path = getPath(it)
            traj = traj.concatenate(path)
        }
        return traj
    }

    fun getPath(name: String): Trajectory {
        val jsonFile = File("$TRAJECTORY_PATH/$name.wpilib.json")
        return TrajectoryUtil.deserializeTrajectory(jsonFile.readText())
    }
    operator fun get(s: String) = getPath(s)
    val routines
        get() = File(AUTO_PATH).list()
    val paths
        get() = File(TRAJECTORY_PATH).list()?.map { it.removeSuffix(".wpilib.json") }
}

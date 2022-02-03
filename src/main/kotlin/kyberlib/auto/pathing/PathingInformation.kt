package kyberlib.auto.pathing

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import kyberlib.command.Debug
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Interal class containing all of the information for Informed RRT*.
 * Used by Pathfinder.kt
 * @param startPosition the start point of pathing
 * #
 */
internal class PathingInformation(val startPosition: Translation2d, val endPosition: Translation2d) : Debug {
    /** The center of the two endpoints */
    val center: Translation2d = startPosition.plus(endPosition).div(2.0)  // average
    /** The distance between the two end nodes of the pathing*/
    private val dis = startPosition.getDistance(endPosition)
    /** Change in location between the two points */
    private val shifted = endPosition.minus(startPosition)
    /** Rotation of the Information Bounding Oval */
    val rotation = Rotation2d(shifted.x, shifted.y)
    /** The shortest found path between the two points */
    var currentPathLength = -1.0
    /** Width of the Information Bounding Oval */
    val width: Double
        get() = currentPathLength
    /** Height of the Information Bounding Oval */
    val height: Double
        get() = sqrt(currentPathLength * currentPathLength - dis * dis)
    /** Whether the pathing information has been updated with a path */
    val pathFound: Boolean
        get() = currentPathLength > 0.0

    /**
     * Update the information with a new path
     * @param currentPathLength the shortest distance found to path between the two points
     */
    fun update(bestPathLength: Double) {
        println("pathLen: $bestPathLength")
        this.currentPathLength = bestPathLength.coerceAtLeast(dis)
    }

    /**
     * Converts the polar coordinates of the oval into Cartesian
     * @param rho the radius of the polar coordinates
     * @param theta the angle of the polar coordinates
     * @return a position representing the Cartesian Coordinates in the field
     */
    fun get(rho: Double, theta: Double): Translation2d {
        assert(pathFound) {"You should not sample from information until pathLength is set"}
        val x = cos(theta) * width/2 * rho
        val y = sin(theta) * height/2 * rho
        val rotated = Translation2d(x, y).rotateBy(rotation)
        return rotated.plus(center)
    }

    /**
     * Print all the values for debugging purposes.
     * Should not be used during competition
     */
    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "start" to startPosition,
            "end" to endPosition,
            "best path" to currentPathLength
        )
    }
}
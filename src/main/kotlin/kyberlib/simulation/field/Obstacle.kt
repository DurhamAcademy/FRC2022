package kyberlib.simulation.field

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import java.awt.geom.Line2D
import java.awt.geom.Point2D
import java.awt.geom.Rectangle2D

/**
 * A rectangular obstacle on the field
 *
 * @param pose the position and orientation of the obstacle
 * @param width horizontal distance from the center to the side (meters)
 * @param height vertical distance from center to top/bottom (meters)
 */
class Obstacle(val pose: Pose2d, val width: Double, val height: Double) {
    val x: Double
        get() = pose.x
    val y: Double
        get() = pose.y
    val rotation: Rotation2d
        get() = pose.rotation
    val position: Translation2d
        get() = pose.translation
    private val box = Rectangle2D.Double(x-width, y-height, width*2, height*2)

    init {
        KField2d.getObject("obstacles").poses.add(pose)
    }

    /**
     * Checks if a point falls within the obstacles hitbox
     */
    fun contains(point: Translation2d): Boolean {
//        val adjusted = normalize(point)
        val adjusted = point
        val point2d = Point2D.Double(adjusted.x, adjusted.y)
        return box.contains(point2d)
    }

    /**
     * Whether obstacle contains line between two points
     */
    fun contains(point1: Translation2d, point2: Translation2d): Boolean {
//        val adj1 = normalize(point1)
//        val adj2 = normalize(point2)
        val adj1 = point1
        val adj2 = point2
        val line = Line2D.Double(adj1.x, adj1.y, adj2.x, adj2.y)
        return line.intersects(box)
    }


    private fun normalize(point: Translation2d): Translation2d {
        val centered = point.minus(position)
        return centered.rotateBy(-rotation)
    }

    override fun toString(): String {
        return "Obstacle @ ($x, $y) and dims ($width, $height)"
    }

}
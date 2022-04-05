package frc.kyberlib.simulation.field

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import frc.kyberlib.math.units.extensions.Length
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.w
import java.awt.Polygon
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
class Obstacle(vararg val points: Translation2d) {
    companion object {
        private const val scale = 1000
    }
    constructor(pos: Translation2d, width: Length, height: Length) : this(
        Translation2d(pos.x-width.value, pos.y+height.value), Translation2d(pos.x-width.value, pos.y-height.value),
        Translation2d(pos.x+width.value, pos.y+height.value), Translation2d(pos.x+width.value, pos.y-height.value)
    )
    val xPoints = points.map { it.x.toInt() * 1000 }.toIntArray()
    val yPoints = points.map { it.y.toInt() * 1000 }.toIntArray()

    val centerPoint = Translation2d(xPoints.average() / 1000, yPoints.average() / 1000)

    val poly = Polygon(xPoints, yPoints, points.size)

    init {
        KField2d.getObject("obstacles").poses.add(Pose2d(centerPoint, 0.degrees.w))
    }

    /**
     * Checks if a point falls within the obstacles hitbox
     */
    fun contains(point: Translation2d): Boolean {
        val point2d = Point2D.Double(point.x * 1000, point.y * 1000)
        return poly.contains(point2d)
    }

    /**
     * Whether obstacle contains line between two points
     */
    fun contains(point1: Translation2d, point2: Translation2d): Boolean {
        val line = Line2D.Double(point1.x * 1000, point1.y * 1000, point2.x * 1000, point2.y * 1000)
        return line.intersects(poly.bounds2D)
    }

    override fun toString(): String {
        return "Obstacle @ $points"
    }

}
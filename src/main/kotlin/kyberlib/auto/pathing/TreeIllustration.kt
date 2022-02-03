package kyberlib.auto.pathing

import kyberlib.math.units.extensions.meters
import kyberlib.simulation.field.KField2d
import java.awt.Color
import java.awt.Graphics
import java.awt.Graphics2D
import java.awt.geom.Ellipse2D
import javax.swing.JFrame
import javax.swing.JPanel


/**
 * Draws a basic Tree without showing the obstacles
 * @param tree
 */
internal class TreeIllustration(val tree: Tree) : JPanel() {
    constructor(pathfinder: Pathfinder) : this(pathfinder.tree) {
        information = pathfinder.information
        if (pathfinder.pathFound)
            path = pathfinder.path!!
    }
    val field = KField2d
    var information: PathingInformation? = null
    var path: Collection<Node> = emptySet()

    val frameWidth = 200
    val frameHeight = 200

    /** Whether to draw the path */
    var drawPath = true
    /** Whether to draw the Bounding Oval - shows the limits of where new nodes where helpful */
    var drawBound = true
    /** Whether to draw the obstacles */
    var drawObstacles = true
    /** Whether to draw the normal path nodes */
    var drawNormal = true

    /**
     * Show the drawing
     */
    fun appear() {
        val frame = JFrame()
        frame.setSize(frameWidth, frameHeight)
        frame.title = "Tree drawing"
        frame.layout = null
        frame.defaultCloseOperation = JFrame.EXIT_ON_CLOSE
        frame.contentPane = this
        frame.isVisible = true
    }

    public override fun paintComponent(g: Graphics) { draw(g as Graphics2D) }

    /**
     * Puts the branches onto a graphics window
     */
    private fun draw(graphics: Graphics2D) {
        graphics.color = Color.BLACK
        if (drawObstacles) drawObstacles(graphics)
        if (drawNormal || drawPath) drawBranch(tree.vertices[0], graphics)
        if (information != null) {
            val start = information!!.startPosition
            val end = information!!.endPosition
            graphics.color = Color.GREEN
            // draw startPoint
            graphics.drawOval(drawingCoordinates(start.x), drawingCoordinates(start.y), 10, 10)
            // draw endPoint
            graphics.drawOval(drawingCoordinates(end.x), drawingCoordinates(end.y), 10, 10)
            if (information != null && information!!.currentPathLength > 0.0 && drawBound) drawPathOval(graphics)
        }
    }

    /**
     * Draw the obstacles on the field
     */
    private fun drawObstacles(graphics: Graphics2D) {
        for (obstacle in field.obstacles) {
            graphics.drawRect(drawingCoordinates(obstacle.x-obstacle.width), drawingCoordinates(obstacle.y-obstacle.height), drawingCoordinates(obstacle.width*2), drawingCoordinates(obstacle.height*2))
        }
    }

    /**
     * Draws the oval for Informed RRT*
     * Any point outside of this oval cannot be useful
     */
    private fun drawPathOval(graphics: Graphics2D) {
        val center = information!!.center
        val width = information!!.width
        val height = information!!.height
        val oval = Ellipse2D.Double(drawingCoordinates(center.x-width/2.0).toDouble(), drawingCoordinates(center.y-height/2).toDouble(), drawingCoordinates(width).toDouble(), drawingCoordinates(height).toDouble())
        graphics.color = Color.BLUE
        graphics.rotate(
            information!!.rotation.radians, drawingCoordinates(center.x).toDouble(), drawingCoordinates(
                information!!.center.y).toDouble())
        graphics.draw(oval)
    }

    /**
     * Recursively draws each part of the branch
     */
    private fun drawBranch(n: Node, g: Graphics2D) {
        val x1 = drawingCoordinates(n.position.x)
        val y1 = drawingCoordinates(n.position.y)
        for (n2 in n.children) {
            val x2 = drawingCoordinates(n2.position.x)
            val y2 = drawingCoordinates(n2.position.y)
            val pathNode = path.contains(n2)
            if (drawPath && pathNode) g.color = Color.RED
            else g.color = Color.BLACK
            if ((drawPath || drawNormal) && pathNode) g.drawLine(x1, y1, x2, y2)
            else if (drawNormal && !pathNode) g.drawLine(x1, y1, x2, y2)
            drawBranch(n2, g)
        }
    }

    /**
     * Convert the field coordinates to helpful pixel values
     */
    private fun drawingCoordinates(mapValue: Double) = (mapValue / field.width.meters * 1000).toInt()
}

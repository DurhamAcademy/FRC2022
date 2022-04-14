package frc.kyberlib.sensors

import edu.wpi.first.networktables.NetworkTableInstance
import frc.kyberlib.math.threeD.Pose3d
import frc.kyberlib.math.threeD.Rotation3d
import frc.kyberlib.math.threeD.Translation3d
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.math.units.extensions.milliseconds
import org.photonvision.targeting.PhotonTrackedTarget

/**
 * A wrapper for the limelight vision camera.
 * [table] defines the network table the data will be pulled from ("limelight" by default)
 */
class Limelight(private val table: String = "limelight") {

    enum class LedMode(val idx: Int) {
        PIPELINE(0),
        FORCE_OFF(1),
        FORCE_BLINK(2),
        FORCE_ON(3)
    }

    enum class StreamMode(val idx: Int) {
        STANDARD(0),
        PIP_PRIMARY(1),
        PIP_SECONDARY(2)
    }

    private val tbl = NetworkTableInstance.getDefault().getTable(table)

    private val foundEntry = tbl.getEntry("tb")
    /**
     * Is the camera currently detecting an object?
     */
    val targetFound: Boolean
        get() = foundEntry.getDouble(0.0) != 0.0

    private val yawEntry = tbl.getEntry("tx")
    /**
     * The x-heading the detection is in
     */
    val yaw
        get() = yawEntry.getDouble(0.0).degrees

    private val pitchEntry = tbl.getEntry("ty")
    /**
     * The y-heading the detection is in
     */
    val pitch
        get() = pitchEntry.getDouble(0.0).degrees

    private val areaEntry = tbl.getEntry("ta")
    /**
     * The area of the detected contour
     */
    val area
        get() = areaEntry.getDouble(0.0)

    private val skewEntry = tbl.getEntry("ts")
    /**
     * The angle of the detected contour relative to the camera
     */
    val skew
        get() = skewEntry.getDouble(0.0)

    private val latencyEntry = tbl.getEntry("tl")
    /**
     * The milliseconds it takes for the pipeline to be processed
     */
    val latency
        get() = latencyEntry.getDouble(0.0).milliseconds

    private val shortEntry = tbl.getEntry("tl")
    /**
     * Sidelength of shortest side of the fitted bounding box (pixels)
     */
    val short
        get() = shortEntry.getDouble(0.0)

    private val longEntry = tbl.getEntry("tlong")
    /**
     * 	Sidelength of longest side of the fitted bounding box (pixels)
     */
    val long
        get() = longEntry.getDouble(0.0)

    private val horEntry = tbl.getEntry("thor")
    /**
     * Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     */
    val horizontal
        get() = horEntry.getDouble(0.0)

    private val vertEntry = tbl.getEntry("tvert")
    /**
     * Vertical sidelength of the rough bounding box (0 - 320 pixels)
     */
    val vertical
        get() = vertEntry.getDouble(0.0)

    private val ledEntry = tbl.getEntry("ledMode")
    /**
     * Sets limelight’s LED state
     */
    var ledMode
        get() = LedMode.values().find { it.idx == ledEntry.getNumber(0) } ?: LedMode.PIPELINE
        set(value) {
            ledEntry.setNumber(value.idx)
        }

    private val streamEntry = tbl.getEntry("ledMode")
    /**
     * 	Sets limelight’s streaming mode
     */
    var stream
        get() = StreamMode.values().find { it.idx == streamEntry.getNumber(0) } ?: StreamMode.STANDARD
        set(value) {
            streamEntry.setNumber(value.idx)
        }

    private val driverEntry = tbl.getEntry("camMode")
    /**
     * Sets limelight’s operation mode. True enables Driver Camera (Increases exposure, disables vision processing)
     */
    var driverMode: Boolean
        get() = driverEntry.getNumber(0) == 1
        set(value) {
            driverEntry.setNumber(if (value) 1 else 0)
        }

    private val snapEntry = tbl.getEntry("snapshot")
    /**
     * Allows users to take snapshots during a match
     */
    var snapshot: Boolean
        get() = snapEntry.getNumber(0) == 1
        set(value) {
            snapEntry.setNumber(if (value) 1 else 0)
        }

    /**
     * Active pipeline index of the camera (0 .. 9)
     */
    var pipeline
        get() = tbl.getEntry("getpipe").getDouble(0.0).toInt()
        set(value) {
            tbl.getEntry("pipeline").setNumber(value)
        }

    private val tranEntry = tbl.getEntry("camtran")
    /**
     * 3D transform of the detected target if using PnP
     */
    val transform: Pose3d?
        get() {
            val entry = tranEntry
            val cmt = if (entry.exists()) entry.getDoubleArray(arrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)) else null
            return if (cmt != null)
                Pose3d(Translation3d(cmt[0].meters, cmt[1].meters, cmt[2].meters),
                        Rotation3d(cmt[3].degrees, cmt[4].degrees, cmt[5].degrees)) else null
        }

    /**
     * Copy of Photons sim support for limelight
     */
    fun submitProcessedFrame(latencyMils: Double, targets: List<PhotonTrackedTarget>) {
        latencyEntry.setNumber(latencyMils)
        if(targets.isEmpty()) {
            foundEntry.setNumber(0.0)
            return
        }
        val target = targets.first() // limelight only supports one I think
        yawEntry.setNumber(target.yaw)
        pitchEntry.setNumber(target.pitch)
        areaEntry.setNumber(target.area)
        skewEntry.setNumber(target.skew)

        val corners = target.corners
        val horizontal = corners[1].x - corners[0].x
        val vertical = corners[3].y - corners[0].y
        horEntry.setNumber(horizontal)
        vertEntry.setNumber(vertical)
        longEntry.setNumber(minOf(horizontal, vertical))
        shortEntry.setNumber(maxOf(horizontal, vertical))
    }
}

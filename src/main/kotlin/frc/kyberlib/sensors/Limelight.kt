package frc.kyberlib.sensors

import edu.wpi.first.networktables.NetworkTableInstance
import frc.kyberlib.math.threeD.Pose3d
import frc.kyberlib.math.threeD.Rotation3d
import frc.kyberlib.math.threeD.Translation3d
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.math.units.extensions.milliseconds

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

    /**
     * Is the camera currently detecting an object?
     */
    val targetFound: Boolean
        get() = tbl.getEntry("tv").getDouble(0.0) != 0.0

    /**
     * The x-heading the detection is in
     */
    val yaw
        get() = tbl.getEntry("tx").getDouble(0.0).degrees

    /**
     * The y-heading the detection is in
     */
    val pitch
        get() = tbl.getEntry("ty").getDouble(0.0).degrees

    /**
     * The area of the detected contour
     */
    val area
        get() = tbl.getEntry("ta").getDouble(0.0)

    /**
     * The angle of the detected contour relative to the camera
     */
    val skew
        get() = tbl.getEntry("ts").getDouble(0.0)

    /**
     * The milliseconds it takes for the pipeline to be processed
     */
    val latency
        get() = tbl.getEntry("tl").getDouble(0.0).milliseconds

    /**
     * Sidelength of shortest side of the fitted bounding box (pixels)
     */
    val short
        get() = tbl.getEntry("tshort").getDouble(0.0)

    /**
     * 	Sidelength of longest side of the fitted bounding box (pixels)
     */
    val long
        get() = tbl.getEntry("tlong").getDouble(0.0)

    /**
     * Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     */
    val horizontal
        get() = tbl.getEntry("thor").getDouble(0.0)

    /**
     * Vertical sidelength of the rough bounding box (0 - 320 pixels)
     */
    val vertical
        get() = tbl.getEntry("tvert").getDouble(0.0)

    /**
     * Sets limelight’s LED state
     */
    var ledMode
        get() = LedMode.values().find { it.idx == tbl.getEntry("ledMode").getNumber(0) } ?: LedMode.PIPELINE
        set(value) {
            tbl.getEntry("ledMode").setNumber(value.idx)
        }

    /**
     * 	Sets limelight’s streaming mode
     */
    var stream
        get() = StreamMode.values().find { it.idx == tbl.getEntry("stream").getNumber(0) } ?: StreamMode.STANDARD
        set(value) {
            tbl.getEntry("stream").setNumber(value.idx)
        }

    /**
     * Sets limelight’s operation mode. True enables Driver Camera (Increases exposure, disables vision processing)
     */
    var driverMode: Boolean
        get() = tbl.getEntry("camMode").getNumber(0) == 1
        set(value) {
            tbl.getEntry("camMode").setNumber(if (value) 1 else 0)
        }

    /**
     * Allows users to take snapshots during a match
     */
    var snapshot: Boolean
        get() = tbl.getEntry("snapshot").getNumber(0) == 1
        set(value) {
            tbl.getEntry("snapshot").setNumber(if (value) 1 else 0)
        }

    /**
     * Active pipeline index of the camera (0 .. 9)
     */
    var pipeline
        get() = tbl.getEntry("getpipe").getDouble(0.0).toInt()
        set(value) {
            tbl.getEntry("pipeline").setNumber(value)
        }

    /**
     * 3D transform of the detected target if using PnP
     */
    val transform: Pose3d?
        get() {
            val entry = tbl.getEntry("camtran")
            val cmt = if (entry.exists()) entry.getDoubleArray(arrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)) else null
            return if (cmt != null)
                Pose3d(Translation3d(cmt[0].meters, cmt[1].meters, cmt[2].meters),
                        Rotation3d(cmt[3].degrees, cmt[4].degrees, cmt[5].degrees)) else null
        }
}

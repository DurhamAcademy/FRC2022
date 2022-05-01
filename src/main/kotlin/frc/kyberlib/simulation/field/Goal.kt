package frc.kyberlib.simulation.field

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.w

/**
 * A custom field object that has a angle and linked command
 */
class Goal(val name: String, val position: Translation2d, private val uponArrival: Command? = null) {  // this is still kinda janky
    private val fieldObject: FieldObject2d
        get() = KField2d.getObject(name)

    init { add() }

    /**
     * Adds the goal to field
     */
    private fun add() {
        val prevPoses = fieldObject.poses
        prevPoses.add(Pose2d(position, 0.degrees.w))
        fieldObject.poses = prevPoses
        KField2d.goals.add(this)
    }

    /**
     * Remove from field
     */
    private fun remove() {
        val prevPoses = fieldObject.poses
        prevPoses.remove(Pose2d(position, 0.degrees.w))
        fieldObject.poses = prevPoses
        KField2d.goals.remove(this)
    }
    /**
     * Command to execute when trying to complete this gaol
     */
    val command: Command
        get() {
//            val pathCommand = AutoDrive(angle)
//            if (uponArrival != null) return pathCommand.andThen(uponArrival).andThen(this::remove, Drivetrain)
//            return pathCommand.andThen(this::remove, Drivetrain)
            return InstantCommand()
        }

}
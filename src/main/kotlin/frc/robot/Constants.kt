package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import frc.kyberlib.math.Interpolator
import frc.kyberlib.math.units.MassConversions
import frc.kyberlib.math.units.Pose2d
import frc.kyberlib.math.units.Translation2d
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.zeroPose
import frc.kyberlib.simulation.field.KField2d

/**
 * This file holds all important constants throughout the project
 */
object Constants {
    const val doStateSpace = false
    // ------ Climb ------ //
    val WINCH_RADIUS: Length = 0.6.inches
    const val EXTENDABLE_ROTATION_GEAR_RATIO = 1.0 / 250.0
    const val WINCH_GEAR_RATIO = (12.0 / 62.0) * (18.0 / 34.0) * (36.0 / 54.0)

    // ------ Conveyor ------ //

    // ------ Drivetrain ------ //
    // pids
    const val DRIVE_P = 0.0//0.5
    const val DRIVE_I = 0.0
    const val DRIVE_D = 0.0


    // feed forwards
    const val DRIVE_KS_R = 0.28923
    const val DRIVE_KV_R = 2.6081
    const val DRIVE_KA_R = 0.16727

    const val DRIVE_KS_L = 0.26653
    const val DRIVE_KV_L = 2.5835
    const val DRIVE_KA_L = 0.472

    // drivetrain setup
    const val TRACK_WIDTH = 0.72246
    val WHEEL_RADIUS = 2.inches
    const val DRIVE_GEAR_RATIO = (10.0 / 50.0) * (50.0 / 62.0)

    const val NAVIGATION_CORRECTION = false

    // ------ Intake ------ //
    const val INTAKE_PERCENT = 0.5

    // ------ Shooter ------ //
    val LIMELIGHT_HEIGHT: Length = 23.216.inches
    val LIMELIGHT_ANGLE: Angle = 50.degrees

    // TODO: test shots
    val FLYWHEEL_INTERPOLATOR: Interpolator = Interpolator(mapOf(
        100.inches.meters to 3.0,
        200.inches.meters to 5.0,
        300.inches.meters to 7.0
    ))
    val TOPWHEEL_INTERPOLATOR: Interpolator = Interpolator(mapOf(
        100.inches.meters to 1.0,
        200.inches.meters to 2.0,
        300.inches.meters to 3.0
    ))
    val HOODANGLE_INTERPOLATOR: Interpolator = Interpolator(mapOf(
        100.inches.meters to 80.0,
        200.inches.meters to 60.0,
        300.inches.meters to 4.0
    ))

    val SHOOTER_VELOCITY_TOLERANCE = 50.0.rpm
    val FLYWHEEL_RADIUS = 2.inches

    // ------ Turret ------ //
    val TURRET_GEAR_RATIO: Double = (1.0 / 10.0) * (18.0 / 215.0)
    const val NOT_FOUND_WAIT = 0.2
    const val LOST_WAIT = 2.0
    const val SMART_LOSS = false

    const val SHOOTER_AQUISITION_TIME = 0.2
    val TURRET_TOLERANCE = 3.degrees // how close the heading must be to fire
    val TURRET_DEADBAND = 2.degrees  // when the turret stops adjusting

    // ------ Game Pieces ------ //
    val HUB_POSITION = Translation2d(324.inches, 162.inches)
    val FIELD_SIZE = Translation2d(648.inches, 324.inches)
    val UPPER_HUB_HEIGHT = 8.feet + 8.inches
    val START_POSE: Pose2d = Pose2d(8.5.meters, 1.9.meters, (-90).degrees)

    val LOW_RUNG_HEIGHT = 4.feet + .75.inches
    val MID_RUNG_HEIGHT = 5.feet + .25.inches
    val LOW2MID = 3.feet + 6.inches
    val HIGH_RUNG_HEIGHT = 6.feet + 3.inches
    val MID2HIGH = 2.feet
    val TRAVERSAL_RUNG_HEIGHT = 7.feet + 7.inches
    val HIGH2TRAVERSE = 2.feet

    // ----- measurements ----- //
    val ROBOT_WEIGHT = 120.0 * MassConversions.poundsToGrams * 1000.0
    const val CLIMB_MOMENT_OF_INERTIA = 0.10571
    const val TOP_ROLLER_MOMENT_OF_INERTIA = 0.00001
    const val FLYWHEEL_MOMENT_OF_INERTIA = 0.00064 // kg * m^2
}
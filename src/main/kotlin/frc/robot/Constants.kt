package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import frc.kyberlib.math.Interpolator
import frc.kyberlib.math.units.MassConversions
import frc.kyberlib.math.units.Translation2d
import frc.kyberlib.math.units.extensions.*

/**
 * This file holds all important constants throughout the project
 */
object Constants {
    const val doStateSpace = false  // todo remove this

    // ------ Climb ------ // todo: update these
    val WINCH_RADIUS: Length = 0.5.inches
    const val EXTENDABLE_ROTATION_GEAR_RATIO = 250.0
    const val WINCH_GEAR_RATIO = (10.0 / 1.0) * (54.0 / 18.0)

    // ------ Conveyor ------ //
    const val BALL_TRACKING = false
    const val DRIVER_CAMERA = false

    // ------ Drivetrain ------ //
    // pids
    const val DRIVE_P = 0.03
    const val DRIVE_I = 0.0
    const val DRIVE_D = 0.0


    // feed forwards
    const val DRIVE_KS_R = 0.16983
    const val DRIVE_KV_R = 2.472
    const val DRIVE_KA_R = 0.14531

    const val DRIVE_KS_L = 0.15264
    const val DRIVE_KV_L = 2.4928
    const val DRIVE_KA_L = 0.15796

    // drivetrain setup
    const val TRACK_WIDTH = 0.64
    val WHEEL_RADIUS = 2.inches
    const val DRIVE_GEAR_RATIO = 62.0 / 10.0

    // ------ Intake ------ //
    const val INTAKE_PERCENT = 0.8

    // ------ Shooter ------ //
    val LIMELIGHT_HEIGHT: Length = 25.inches
    val LIMELIGHT_ANGLE: Angle = 40.degrees
    const val MOVEMENT_CORRECTION = true

    val FLYWHEEL_INTERPOLATOR: Interpolator = Interpolator(
        mapOf(
            1.7 to 1400.0,
            2.09375 to 1440.0,
            2.4875 to 1420.0,
            2.88125 to 1470.0,
            3.275 to 1500.0,
            3.66875 to 1600.0,
            4.0625 to 1610.0,
            4.45625 to 1730.0,
            4.85 to 1800.0
        )
    )
    val HOODANGLE_INTERPOLATOR: Interpolator = Interpolator(
        mapOf(
            1.7 to 15.0,
            2.09375 to 28.0,
            2.4875 to 30.0,
            2.88125 to 36.0,
            3.275 to 40.0,
            3.66875 to 45.0,
            4.0625 to 50.0,
            4.45625 to 50.0,
            4.85 to 50.0
        )
    )

    val TIME_OF_FLIGHT_INTERPOLATOR = Interpolator(mapOf(
        2.5 to 1.0,
        3.0 to 1.15,
        3.5 to 1.15,
        4.0 to 1.09,
        4.5 to 1.32,
        5.0 to 1.4,
        5.5 to 1.52
    ))

    val SHOOTER_VELOCITY_TOLERANCE = 50.rpm

    // ------ Turret ------ //
    const val TURRET_GEAR_RATIO: Double = 10.0 * (215.0 / 18.0)  // 10 is vp and second is pulley ratio

    val TURRET_TOLERANCE = 3.degrees // how close the heading must be to fire


    // ------ Game Pieces ------ //
    val HUB_POSITION = Translation2d(324.inches, 162.inches)
    val FIELD_SIZE = Translation2d(648.inches, 324.inches)
    val UPPER_HUB_HEIGHT = 8.feet + 8.inches
    val START_POSE: Pose2d = Pose2d(
        HUB_POSITION - Translation2d(3.meters, 0.1.meters),
        0.degrees.w
    )
    val RESET_POSE = Pose2d(1.3, 1.3, 45.degrees.w) // center of our terminal

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
    val BALL_DIAMETER = 9.5.inches
}
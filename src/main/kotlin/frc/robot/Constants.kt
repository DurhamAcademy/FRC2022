package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import frc.kyberlib.math.Interpolator
import frc.kyberlib.math.units.MassConversions
import frc.kyberlib.math.units.Translation2d
import frc.kyberlib.math.units.extensions.*

/**
 * This file holds all important constants throughout the project
 */
object Constants {  // todo
    // basically check every electrical compenent is accounted for and update gear ratios of closed loop motors
    const val DRIVE_ONLY = true
    // ------ Climb ------ //
    val WINCH_RADIUS: Length = 0.39.inches
    const val WINCH_GEAR_RATIO = (10.0 / 1.0) * (54.0 / 18.0)  // fixme

    // ------ Drivetrain ------ //
    // pids
    const val DRIVE_P = 0.03
    const val DRIVE_I = 0.0
    const val DRIVE_D = 0.0


    // feed forwards  // fixme
    const val DRIVE_KS = 0.16983
    const val DRIVE_KV = 2.472
    const val DRIVE_KA = 0.14531

    const val TURN_KS = 0.0
    const val TURN_KV = 0.0
    const val TURN_KA = 0.0

    // drivetrain setup
    const val DRIVE_GEAR_RATIO = 62.0 / 10.0
    const val TURN_GEAR_RATIO = 1.0

    // ------ Intake ------ //
    const val INTAKE_PERCENT = 0.8

    // ------ Shooter ------ //
    val LIMELIGHT_HEIGHT: Length = 25.inches
    val LIMELIGHT_ANGLE: Angle = 40.degrees

    // fixme
    val FLYWHEEL_INTERPOLATOR: Interpolator = Interpolator(
        mapOf(
            1.84 to 1300.0,
            2.39 to 1350.0,
            2.94 to 1420.0,
            3.48 to 1470.0,
            4.02 to 1550.0,
            4.65 to 1640.0,
            5.11 to 1720.0,
            5.565 to 1800.0,
            6.2 to 1900.0
        )
    )
    val HOODANGLE_INTERPOLATOR: Interpolator = Interpolator(
        mapOf(
            1.84 to 20.0,
            2.39 to 33.0,
            2.94 to 42.0,
            3.48 to 49.0,
            4.02 to 53.0,
            4.65 to 63.0,
            5.11 to 65.0,
            5.565 to 70.0,
            6.2 to 75.0
        )
    )

    val TIME_OF_FLIGHT_INTERPOLATOR = Interpolator(
        mapOf(
            2.5 to 1.0,
            3.0 to 1.05,
            3.5 to 1.10,
            4.0 to 1.15,
            4.5 to 1.32,
            5.0 to 1.4,
            5.5 to 1.52
        )
    )

    val SHOOTER_VELOCITY_TOLERANCE = 20.rpm

    // ------ Game Pieces ------ //
    val HUB_POSITION = Translation2d(324.inches, 162.inches)
    val FIELD_SIZE = Translation2d(648.inches, 324.inches)
    val UPPER_HUB_HEIGHT = 8.feet + 8.inches
    val START_POSE: Pose2d = Pose2d(
        HUB_POSITION - Translation2d(3.meters, 0.1.meters),
        0.degrees.w
    )

    // ----- measurements ----- //
    val ROBOT_WEIGHT = 115.0 * MassConversions.poundsToGrams * 1000.0  // kg
}
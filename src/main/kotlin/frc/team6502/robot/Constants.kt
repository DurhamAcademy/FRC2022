package frc.team6502.robot

import edu.wpi.first.wpilibj.geometry.Pose2d
import kyberlib.math.Interpolator
import kyberlib.math.units.Translation2d
import kyberlib.math.units.extensions.*

/**
 * This file holds all important constants throughout the project
 */
object Constants {
    // ------ Climb ------ //
    val WINCH_RADIUS: Length = TODO()
    const val WINCH_GEAR_RATIO = 0.0 // TODO

    // ------ Conveyor ------ //

    // ------ Drivetrain ------ // todo: characterize
    // pids
    const val DRIVE_P = 0.0
    const val DRIVE_I = 0.0
    const val DRIVE_D = 0.0

    // feed forwards
    const val DRIVE_KS = 0.0
    const val DRIVE_KV = 0.0
    const val DRIVE_KA = 0.0

    // drivetrain setup - todo: tune
    const val TRACK_WIDTH = 0.657299651
    val WHEEL_RADIUS = 2.inches  // todo: measure
    const val DRIVE_GEAR_RATIO = (10.0 / 50.0) * (50.0 / 62.0)

    // ------ Intake ------ //
    const val INTAKE_PERCENT = 0.5

    // ------ Shooter ------ //
    val LIMELIGHT_HEIGHT: Length = TODO()
    val LIMELIGHT_ANGLE: Angle = TODO()

    val FLYWHEEL_INTERPOLATOR: Interpolator = TODO()
    val TOPWHEEL_INTERPOLATOR: Interpolator = TODO()
    val HOODANGLE_INTERPOLATOR: Interpolator = TODO()

    val SHOOTER_VELOCITY_TOLERANCE = 50.0.rpm
    val FLYWHEEL_RADIUS = 2.inches

    // ------ Turret ------ //
    const val TURRET_GEAR_RATIO: Double = 0.0 // TODO
    const val NOT_FOUND_WAIT = 0.2
    const val LOST_WAIT = 2.0

    val TURRET_TOLERANCE = 3.degrees // TODO - how close the heading must be to fire
    val TURRET_DEADBAND = 0.5.degrees

    // ------ GamePieces ------ //
    val HUB_POSITION = Translation2d(162.inches, 324.inches)
    val UPPER_HUB_HEIGHT = 8.feet + 8.inches
    val START_POSE: Pose2d = TODO()
}
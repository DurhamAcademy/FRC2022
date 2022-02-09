package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import frc.kyberlib.math.Interpolator
import frc.kyberlib.math.units.Pose2d
import frc.kyberlib.math.units.Translation2d
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.zeroPose
import frc.kyberlib.simulation.field.KField2d

/**
 * This file holds all important constants throughout the project
 */
object Constants {
    // ------ Climb ------ //
    val WINCH_RADIUS: Length = 1.inches  // TODO
    const val WINCH_GEAR_RATIO = 1.0 // TODO

    // ------ Conveyor ------ //

    // ------ Drivetrain ------ // todo: characterize
    // pids
    const val DRIVE_P = 0.7
    const val DRIVE_I = 0.0
    const val DRIVE_D = 0.0

    // feed forwards
    const val DRIVE_KS = 0.246
    const val DRIVE_KV = 2.59
    const val DRIVE_KA = 0.494

    // drivetrain setup - todo: tune
    const val TRACK_WIDTH = 0.657299651
    val WHEEL_RADIUS = 2.inches  // todo: measure
    const val DRIVE_GEAR_RATIO = (10.0 / 50.0) * (50.0 / 62.0)

    // ------ Intake ------ //
    const val INTAKE_PERCENT = 0.5

    // ------ Shooter ------ //
    val LIMELIGHT_HEIGHT: Length = 36.inches // TODO
    val LIMELIGHT_ANGLE: Angle = 45.degrees // TODO

    val FLYWHEEL_INTERPOLATOR: Interpolator = Interpolator(mapOf())  // TODO
    val TOPWHEEL_INTERPOLATOR: Interpolator = Interpolator(mapOf())  // TODO
    val HOODANGLE_INTERPOLATOR: Interpolator = Interpolator(mapOf())  // TODO

    val SHOOTER_VELOCITY_TOLERANCE = 50.0.rpm
    val FLYWHEEL_RADIUS = 2.inches
    const val FLYWHEEL_MOMENT_OF_INERTIA = 0.00032 // kg * m^2

    // ------ Turret ------ //
    const val TURRET_GEAR_RATIO: Double = 1.0/73.0 // TODO
    const val NOT_FOUND_WAIT = 0.2
    const val LOST_WAIT = 2.0
    const val SMART_LOSS = true

    val TURRET_TOLERANCE = 5.degrees // TODO - how close the heading must be to fire
    val TURRET_DEADBAND = 3.degrees  // when the turret stops adjusting

    // ------ Game Pieces ------ //
    val HUB_POSITION = Translation2d(324.inches, 162.inches)
    val FIELD_SIZE = Translation2d(648.inches, 324.inches)
    val UPPER_HUB_HEIGHT = 8.feet + 8.inches
    val START_POSE: Pose2d =Pose2d(200.inches, 100.inches, 45.degrees)
}
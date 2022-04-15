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
    // ------ Drivetrain ------ //
    // pids
    const val DRIVE_P = 0.03

    // feed forwards
    const val DRIVE_KS_R = 0.16983
    const val DRIVE_KV_R = 2.472
    const val DRIVE_KA_R = 0.14531

    const val DRIVE_KS_L = 0.15264
    const val DRIVE_KV_L = 2.4928
    const val DRIVE_KA_L = 0.15796

    // drivetrain setup
    val WHEEL_RADIUS = 2.inches
    const val DRIVE_GEAR_RATIO = 62.0 / 10.0

    // ------ Intake ------ //
    const val INTAKE_PERCENT = 0.8

    // ------ Shooter ------ //
    val LIMELIGHT_HEIGHT: Length = 25.inches
    val LIMELIGHT_ANGLE: Angle = 40.degrees

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
    val SHOOTER_VELOCITY_TOLERANCE = 20.rpm

    // ------ Turret ------ //
    const val TURRET_GEAR_RATIO: Double = 10.0 * (215.0 / 18.0)  // 10 is vp and second is pulley ratio
    val TURRET_TOLERANCE = 3.degrees // how close the heading must be to fire

    // ------ Game Pieces ------ //
    val HUB_POSITION = Translation2d(324.inches, 162.inches)
    val UPPER_HUB_HEIGHT = 8.feet + 8.inches
    val START_POSE: Pose2d = Pose2d(HUB_POSITION - Translation2d(3.meters, 0.1.meters), 0.degrees.w)
}
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

    // ------ Drivetrain ------ //
    // pids
    const val DRIVE_P = 0.03
    const val DRIVE_I = 0.0
    const val DRIVE_D = 0.0


    // feed forwards  // fixme
    const val DRIVE_KS = 0.0//0.16983
    const val DRIVE_KV = 0.0//2.472
    const val DRIVE_KA = 0.0//0.14531

    const val TURN_KS = 0.0
    const val TURN_KV = 0.0
    const val TURN_KA = 0.0

    // drivetrain setup
    const val DRIVE_GEAR_RATIO = 8.14
    const val TURN_GEAR_RATIO = 12.8

    // ------ Game Pieces ------ //
    val HUB_POSITION = Translation2d(324.inches, 162.inches)
    val START_POSE: Pose2d = Pose2d(
        HUB_POSITION - Translation2d(3.meters, 0.1.meters),
        0.degrees.w
    )

    // ----- measurements ----- //
    val ROBOT_WEIGHT = 80.0 * MassConversions.poundsToGrams / 1000.0  // kg
}
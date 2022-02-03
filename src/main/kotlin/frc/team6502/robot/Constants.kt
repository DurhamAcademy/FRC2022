package frc.team6502.robot

import edu.wpi.first.wpilibj.DriverStation
import kyberlib.math.units.extensions.inches
import kyberlib.math.units.extensions.metersPerSecond

/**
 * This file holds all important constants throughout the project
 */
object Constants{
    val AUTO
        get() = DriverStation.getInstance().isAutonomousEnabled

    // ------ PORTS ------ //

    // drive ports
    const val LEFT_FRONT_ID = 2
    const val LEFT_BACK_ID = 3
    const val RIGHT_FRONT_ID = 4
    const val RIGHT_BACK_ID = 1

    // navigation
    const val PIGEON_PORT = 6

    // shooter ports
    const val INTAKE_ID = 5
    const val SHOOTER_ID = 7

    // ------ CONTROLS ------ //
    // pids
    const val DRIVE_P = 0.0
    const val DRIVE_I = 0.0
    const val DRIVE_D = 0.0

    // feed forwards
    const val DRIVE_KS = 0.0
    const val DRIVE_KV = 0.0
    const val DRIVE_KA = 0.0

    // drivetrain setup
    const val TRACK_WIDTH = 0.657299651
    val WHEEL_RADIUS = (3.25 / 2.0).inches
    const val DRIVE_GEAR_RATIO = 1.0 / 20.0
}
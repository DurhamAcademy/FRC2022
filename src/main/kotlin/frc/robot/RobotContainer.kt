package frc.robot

import frc.kyberlib.auto.Navigator
import frc.kyberlib.input.controller.KXboxController
import frc.kyberlib.sensors.gyros.KPigeon
import frc.robot.subsystems.Drivetrain

/**
 * Contains all Robot subsystems and sensors hehe uwu you cant see this hehe tate isnt paying attention i mean a
 */
object RobotContainer {
    public val controller = KXboxController(0)

    val gyro = KPigeon(6)
    val nav = Navigator(gyro)
    init {
        Drivetrain
    }
}
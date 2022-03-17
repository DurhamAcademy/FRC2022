package frc.kyberlib.command

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.RobotState
import frc.kyberlib.math.units.extensions.seconds
import frc.kyberlib.math.units.micro

object Game {
    val real = RobotBase.isReal()
    val sim = RobotBase.isSimulation()

    val enabled
        get() = RobotState.isEnabled()
    val disabled
        get() = RobotState.isDisabled()
    val AUTO
        get() = RobotState.isAutonomous()
    val OPERATED
        get() = RobotState.isAutonomous()
    val TEST
        get() = RobotState.isTest()
    val STOPPED
        get() = RobotState.isEStopped()

    val brownedOut
        get() = RobotController.isBrownedOut()

    val CAN
        get() = RobotController.getCANStatus()
    val batteryVoltage
        get() = RobotController.getBatteryVoltage()

    val time
        get() = RobotController.getFPGATime().micro.seconds

    val alliance: DriverStation.Alliance = DriverStation.getAlliance()

    val game: String = DriverStation.getGameSpecificMessage()
}
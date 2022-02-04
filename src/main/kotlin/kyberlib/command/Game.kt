package kyberlib.command

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.RobotState

object Game {
    val real
        get() = RobotBase.isReal()
    val sim
        get() = RobotBase.isSimulation()

    val enabled
        get() = RobotState.isEnabled()
    val disabled
        get() = RobotState.isDisabled()
    val AUTO
        get() = RobotState.isAutonomous()
    val OPERATED
        get() = RobotState.isOperatorControl()
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

    val time = RobotController.getFPGATime()
}
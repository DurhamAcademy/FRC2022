package frc.kyberlib.command

import edu.wpi.first.wpilibj.*
import frc.kyberlib.math.units.extensions.seconds
import frc.kyberlib.math.units.micro

object Game {
    val real = RobotBase.isReal()
    val sim = RobotBase.isSimulation()

    val enabled
        inline get() = RobotState.isEnabled()
    val disabled
        inline get() = RobotState.isDisabled()
    val AUTO
        inline get() = RobotState.isAutonomous()
    val OPERATED
        inline get() = RobotState.isTeleop()
    val TEST
        inline get() = RobotState.isTest()
    val STOPPED
        inline get() = RobotState.isEStopped()
    val COMPETITION
        inline get() = DriverStation.isFMSAttached()

    val brownedOut
        inline get() = RobotController.isBrownedOut()

    val CAN
        inline get() = RobotController.getCANStatus()
    val batteryVoltage
        inline get() = RobotController.getBatteryVoltage()

    val time
        inline get() = Timer.getFPGATimestamp().seconds

    val alliance: DriverStation.Alliance = DriverStation.getAlliance()

    val game: String = DriverStation.getGameSpecificMessage()
}
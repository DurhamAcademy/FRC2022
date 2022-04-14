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
    val PRACTICE
        inline get() = DriverStation.getMatchType() == DriverStation.MatchType.Practice
    val QUALIFICATION
        inline get() = DriverStation.getMatchType() == DriverStation.MatchType.Qualification
    val ELIMINATION
        inline get() = DriverStation.getMatchType() == DriverStation.MatchType.Elimination

    val brownedOut
        inline get() = RobotController.isBrownedOut()

    val CAN
        inline get() = RobotController.getCANStatus()
    val batteryVoltage
        inline get() = RobotController.getBatteryVoltage()

    val time
        inline get() = Timer.getFPGATimestamp().seconds
    val bootTime = time
    var startTime = bootTime
    val matchTime
        inline get() = time - startTime

    val alliance: DriverStation.Alliance = DriverStation.getAlliance()

//    val matchName
//        inline get() = DriverStation.getEventName() + DriverStation.getLocation() + DriverStation.getMatchNumber()

    val game: String = DriverStation.getGameSpecificMessage()
}
package frc.kyberlib.command

import edu.wpi.first.wpilibj.*
import frc.kyberlib.math.units.extensions.seconds

object Game {
    val real = RobotBase.isReal()
    val sim = RobotBase.isSimulation()

    inline val enabled
        get() = RobotState.isEnabled()
    inline val disabled
        get() = RobotState.isDisabled()
    inline val AUTO
        get() = RobotState.isAutonomous()
    inline val OPERATED
        get() = RobotState.isTeleop()
    inline val TEST
        get() = RobotState.isTest()
    inline val STOPPED
        get() = RobotState.isEStopped()
    inline val COMPETITION
        get() = DriverStation.isFMSAttached()
    inline val PRACTICE
        get() = DriverStation.getMatchType() == DriverStation.MatchType.Practice
    inline val QUALIFICATION
        get() = DriverStation.getMatchType() == DriverStation.MatchType.Qualification
    inline val ELIMINATION
        get() = DriverStation.getMatchType() == DriverStation.MatchType.Elimination

    inline val brownedOut
        get() = RobotController.isBrownedOut()

    inline val CAN
        get() = RobotController.getCANStatus()
    inline val batteryVoltage
        get() = RobotController.getBatteryVoltage()

    inline val time
        get() = Timer.getFPGATimestamp().seconds
    var startTime = time
    inline val matchTime
        get() = time - startTime//DriverStation.getMatchTime().seconds

    val alliance: DriverStation.Alliance = DriverStation.getAlliance()

//    val matchName
//        inline get() = DriverStation.getEventName() + DriverStation.getLocation() + DriverStation.getMatchNumber()

    inline val message: String get() = DriverStation.getGameSpecificMessage()
}
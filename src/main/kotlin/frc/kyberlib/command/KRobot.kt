package frc.kyberlib.command

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.kyberlib.motorcontrol.KBasicMotorController

open class KRobot() {
    companion object {
        var enabled = false
            private set
        const val period: Double = 0.02
    }

    /**
     * Normal Robot.
     * Executes some default code before calling the main functions
     */
    private inner class KRobotInternal : TimedRobot(period) {

        override fun robotInit() {
            Debug.log("Robot", "robot init", level=DebugLevel.LowPriority)
            for(motor in KBasicMotorController.allMotors) motor.updateValues()
            HAL.report(FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin)
            if (Game.real) LiveWindow.disableAllTelemetry()
            this@KRobot.robotInit()
        }

        override fun robotPeriodic() {
            Debug.log("Robot", "robot periodic", level=DebugLevel.LowPriority)
            CommandScheduler.getInstance().run()
            this@KRobot.robotPeriodic()
        }

        override fun disabledInit() {
            enabled = false
            Debug.log("Robot", "disabled", level=DebugLevel.LowPriority)
            this@KRobot.disabledInit()
        }

        override fun disabledPeriodic() {
            Debug.log("Robot", "disabled periodic", level=DebugLevel.LowPriority)
            this@KRobot.disabledPeriodic()
        }

        override fun disabledExit() {
            Debug.log("Robot", "disabled periodic", level=DebugLevel.LowPriority)
            this@KRobot.disabledExit()
        }

        override fun autonomousInit() {
            if (!enabled) enabledInit()
            enabled = true
            Debug.log("Robot", "auto init", level=DebugLevel.LowPriority)
            this@KRobot.autonomousInit()
        }

        override fun autonomousPeriodic() {
            Debug.log("Robot", "auto periodic", level=DebugLevel.LowPriority)
            this@KRobot.autonomousPeriodic()
        }

        override fun autonomousExit() {
            Debug.log("Robot", "auto exit", level=DebugLevel.LowPriority)
            this@KRobot.autonomousExit()
        }

        override fun teleopInit() {
            if (!enabled) enabledInit()
            enabled = true
            Debug.log("Robot", "teleop init", level=DebugLevel.LowPriority)
            this@KRobot.teleopInit()
        }

        override fun teleopPeriodic() {
            Debug.log("Robot", "teleop periodic", level=DebugLevel.LowPriority)
            this@KRobot.teleopPeriodic()
        }

        override fun teleopExit() {
            Debug.log("Robot", "teleop exit", level=DebugLevel.LowPriority)
            this@KRobot.teleopExit()
        }

        override fun simulationInit() {
            Debug.log("Robot", "sim init", level=DebugLevel.LowPriority)
            this@KRobot.simulationInit()
        }

        override fun simulationPeriodic() {
            Debug.log("Robot", "sim periodic", level=DebugLevel.LowPriority)
            this@KRobot.simulationPeriodic()
        }

        override fun testInit() {
            Debug.log("Robot", "test init", level=DebugLevel.LowPriority)
            this@KRobot.testInit()
        }

        override fun testPeriodic() {
            Debug.log("Robot", "test periodic", level=DebugLevel.LowPriority)
            this@KRobot.testPeriodic()
        }

        override fun testExit() {
            Debug.log("Robot", "test exit", level=DebugLevel.LowPriority)
            this@KRobot.testExit()
        }
    }

    /**
     * The value being wrapped by KRobot
     */
    private val internalRobot = KRobotInternal()

    /**
     * Ran once when the robot starts up
     */
    open fun robotInit() {}

    /**
     * Ran continuously at all times.
     * CommandScheduler is being ran automatically, so no need to call it here.
     */
    open fun robotPeriodic() {}

    /**
     * Ran once when the robot is disabled
     */
    open fun disabledInit() {}

    /**
     * Ran continuously when the robot is disabled
     */
    open fun disabledPeriodic() {}

    open fun disabledExit() {}

    /**
     * Ran once when robot enters auto or teleop, but not both
     */
    open fun enabledInit() {}

    /**
     * Ran once when the robot enters autonomous mode
     */
    open fun autonomousInit() {}

    /**
     * Ran continuously when the robot is in autonomous mode
     */
    open fun autonomousPeriodic() {}

    open fun autonomousExit() {}

    /**
     * Ran once when the robot enters teleoperated mode
     */
    open fun teleopInit() {}

    /**
     * Ran continuously when the robot is in teleoperated mode
     */
    open fun teleopPeriodic() {}

    open fun teleopExit() {}

    /**
     * Run on the start of simulated robot
     */
    open fun simulationInit() {}

    /**
     * Run continuously while Simulating the robot
     */
    open fun simulationPeriodic() {}

    open fun testInit() {}
    open fun testPeriodic() {}
    open fun testExit() {}

    /**
     * Runs the underlying WPILib startup methods. Run this in Main.kt
     */
    fun initialize() {
        RobotBase.startRobot { internalRobot }
    }

}

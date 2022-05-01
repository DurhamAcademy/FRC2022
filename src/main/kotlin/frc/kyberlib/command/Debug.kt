package frc.kyberlib.command

import edu.wpi.first.networktables.NTSendable
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.WidgetType
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.kyberlib.math.units.KUnit

/**
 * Types of ways to print to the output (driverstation)
 * - Print = normal print statement. Will typically remain in console and white
 * - Warn = yellow font and shows in the small outputs
 * - Error = Bright red letters and will show as error
 */
enum class LogMode {
    PRINT, WARN, ERROR
}

/**
 * The level of importance debugging a certain item should have.
 * Can set the project to only send debug values for items with an importance at or above a given level.
 * Allows for control of information levels.
 */
enum class DebugFilter {
    Low, Normal, High, Max
}

/**
 * Inheritable class that grants multiple types of debugging
 */
interface Debug {
    companion object {
        private const val debugging = false
        private val loggingLevel = if (Game.real) DebugFilter.High else DebugFilter.Normal

        fun log(
            identifier: String,
            text: String,
            mode: LogMode = LogMode.PRINT,
            level: DebugFilter = DebugFilter.Normal,
            stacktrace: Boolean = false
        ) {
            if (level < loggingLevel || !debugging) return
            val output = "[$identifier] $text"
            when (mode) {
                LogMode.PRINT -> println(output)
                LogMode.WARN -> DriverStation.reportWarning(output, stacktrace)
                LogMode.ERROR -> DriverStation.reportError(output, stacktrace)
            }
        }
    }


    fun send(label: String, number: Double) {
        if (!debugging || loggingLevel > priority) return
        SmartDashboard.putNumber(label, number)
    }

    fun send(label: String, boolean: Boolean) {
        if (!debugging || loggingLevel > priority) return
        SmartDashboard.putBoolean(label, boolean)
    }

    fun send(label: String, message: String) {
        if (!debugging || loggingLevel > priority) return
        SmartDashboard.putString(label, message)
    }


    /**
     * Prints out all the values in a neat way
     * @param message the output that will be sent to console
     * @param logMode the way the message should appear
     * @see LogMode
     */
    fun log(message: String, logMode: LogMode = LogMode.PRINT, level: DebugFilter = priority) {
        val isError = logMode == LogMode.ERROR
        Companion.log(identifier, message, logMode, level, stacktrace = isError)
    }

    /**
     * The name of the group of values.
     * Defaults to the name of the calling class
     */
    val identifier: String
        get() = javaClass.simpleName

    /**
     * What level this should be debugged at. Allows for quick changing of how much information the console shows
     */
    val priority: DebugFilter
        get() = DebugFilter.Normal
}
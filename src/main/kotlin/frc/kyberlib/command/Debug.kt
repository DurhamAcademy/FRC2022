package frc.kyberlib.command

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.networktables.NTSendable
import frc.kyberlib.math.units.KUnit
import frc.kyberlib.math.units.extensions.Angle

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
enum class DebugLevel {
    LowPriority, NORMAL, HighPriority, MaxPriority  // todo: find a better naming scheme
}
/**
 * Inheritable class that grants multiple types of debugging
 */
interface Debug {
    companion object {
        var debugging = false
        var loggingLevel = DebugLevel.NORMAL

        fun log(identifier:String, text: String, mode: LogMode = LogMode.PRINT, level: DebugLevel = DebugLevel.NORMAL, stacktrace: Boolean = false) {
            if (level < loggingLevel || !debugging) return
            val output = "[$identifier] $text"
            when (mode) {
                LogMode.PRINT -> println(output)
                LogMode.WARN -> DriverStation.reportWarning(output, stacktrace)
                LogMode.ERROR -> DriverStation.reportError(output, stacktrace)
            }
        }
    }

    /**
     * Debugs all the values into a group in the SmartDashboard
     */
    fun debugDashboard(previousPath: String = "", id: String = identifier) {
        if (!debugging || loggingLevel > priority) return
        val map = debugValues()
        sendMapToDashboard(map, "$previousPath/$id")
    }

    /**
     * Takes a map of values and then puts it into the SmartDashboard in the ideal format
     * @param map the set of info to put in the Dashboard. Formatted as a string (id of data), datum
     * @param rootPath the path (seperated by '/') to navigate in Glass. Allows for nested folders
     */
    private fun sendMapToDashboard(map: Map<String, Any?>, rootPath: String) {
        for (info in map) {
            val path = "$rootPath/${info.key}"
            when (info.value) {  // send each datum as their own type
                is Boolean -> SmartDashboard.putBoolean(path, info.value as Boolean)
                is Double -> SmartDashboard.putNumber(path, info.value as Double)
                is Debug -> sendMapToDashboard((info.value as Debug).debugValues(), path)
                is Map<*, *> -> sendMapToDashboard(info.value as Map<String, Any?>, path)
                is NTSendable -> SmartDashboard.putData(path, info.value as NTSendable)
                is Angle -> SmartDashboard.putNumber(path + " (degrees)", (info.value as Angle).degrees)
                is KUnit<*> -> SmartDashboard.putNumber(path + " (${(info.value as KUnit<*>).value})", (info.value as KUnit<*>).value)
                else -> SmartDashboard.putString(path, info.value.toString())
            }
        }
    }


    /**
     * Prints out all the values in a neat way
     * @param message the output that will be sent to console
     * @param logMode the way the message should appear
     * @see LogMode
     */
    fun log(message: String = debugString, logMode: LogMode = LogMode.PRINT, level: DebugLevel = priority) {
        val isError = logMode == LogMode.ERROR
        Companion.log(identifier, message, logMode, level, stacktrace = isError)
        if (isError) throw AssertionError("Check the log for assertion")
    }

    /**
     * Formatting the debug values into an easy to read string
     */
    val debugString: String
        get() {
            val map = debugValues()
            val stringBuilder = StringBuilder()
            stringBuilder.append("$identifier - ")
            for (info in map) {
                val string = if (info.value is Debug) "(${(info.value as Debug).debugString})" else info.value.toString()
                stringBuilder.append("${info.key}: ${string}, ")
            }
            return stringBuilder.toString()
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
    val priority: DebugLevel
        get() = DebugLevel.NORMAL
    /**
     * Function that retrieves the values that need to be debugged
     * @return map of string (name) to value. Numbers, Booleans, and NTSendables are displayed as such
     */
    fun debugValues(): Map<String, Any?>
}
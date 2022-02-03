package kyberlib.command

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Sendable
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

enum class LogMode {
    PRINT, WARN, ERROR
}
/**
 * Inheritable class that grants multiple types of debugging
 */
interface Debug {
    companion object {
        var debugging = true

        /**
         * Sends message to console as warning
         */
        fun logWarning(identifier:String, text: String, stacktrace: Boolean = false) {
            DriverStation.reportWarning("[$identifier] $text", stacktrace)
        }

        /**
         * Logs an error to the driver station window
         */
        fun logError(identifier:String, text: String, stacktrace: Boolean = false) {
            DriverStation.reportError("[$identifier] $text", stacktrace)
        }
    }

    /**
     * Debugs all the values into a group in the SmartDashboard
     */
    fun debugDashboard(previousPath: String = "", id: String = identifier) {
        if (!debugging) return
        val map = debugValues()
        sendMapToDashboard(map, "$previousPath/$id")
    }

    private fun sendMapToDashboard(map: Map<String, Any?>, rootPath: String) {
        for (info in map) {
            val path = "$rootPath/${info.key}"
            when (info.value) {  // send each datum as their own type
                is Boolean -> SmartDashboard.putBoolean(path, info.value as Boolean)
                is Double -> SmartDashboard.putNumber(path, info.value as Double)
                is Debug -> sendMapToDashboard((info.value as Debug).debugValues(), path)
                is Map<*, *> -> sendMapToDashboard(info.value as Map<String, Any?>, path)
                is Sendable -> SmartDashboard.putData(path, info.value as Sendable)
                else -> SmartDashboard.putString(path, info.value.toString())
            }
        }
    }


    /**
     * Prints out all the values in a neat way
     */
    fun debugPrint(logMode: LogMode = LogMode.PRINT) {
        if (!debugging) return
        if (logMode == LogMode.PRINT) println(debugString)
        else if (logMode == LogMode.WARN) logWarning(debugString)
        if (logMode == LogMode.ERROR) logError(debugString)
    }

    private val debugString: String
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

    fun logError(text: String = debugString, stacktrace: Boolean = false) {
        Companion.logError(identifier, text, stacktrace)
    }

    fun logWarning(text: String = debugString, stacktrace: Boolean = false) {
        Companion.logWarning(identifier, text, stacktrace)
    }

    /**
     * The name of the group of values.
     * Defaults to the name of the calling class
     */
    var identifier: String
        get() = javaClass.simpleName
        set(value) = logError("override me, don't set. Interfaces be wack")

    /**
     * Function that retrieves the values that need to be debugged
     * @return map of string (name) to value. Numbers, Booleans, and Sendables are displayed as such
     */
    fun debugValues(): Map<String, Any?>
}
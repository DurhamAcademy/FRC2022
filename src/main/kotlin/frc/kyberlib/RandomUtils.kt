package frc.kyberlib

import java.io.File
import java.io.IOException

// random misc functions

/**
 * Executes string as command in terminal
 * @param workingDir the directory the terminal will be located
 */
fun String.runCommand(workingDir: File) {  // this doesn't work for some reason
    try {
        val parts = this.split("\\s".toRegex())
        val proc = ProcessBuilder(*parts.toTypedArray()).apply {
            directory(workingDir)
            redirectOutput(ProcessBuilder.Redirect.PIPE)
            redirectError(ProcessBuilder.Redirect.PIPE)
        }
        proc.start()
    } catch (e: IOException) {
        e.printStackTrace()
    }
}
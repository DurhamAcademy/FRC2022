package frc.kyberlib.motorcontrol.characterization

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.seconds

object CharacterizationUtil {
    val test: String
        inline get() = SmartDashboard.getString("SysIdTest", "")
    val type: String
        inline get() = SmartDashboard.getString("SysIdTestType", "")
    val voltageCommand: Double
        inline get() = SmartDashboard.getNumber("SysIdVoltageCommand", 0.0)

    var startTime = Game.time
    val baseVoltage: Double
        inline get() = when (type) {
            "Quasistatic" -> voltageCommand * (Game.time - startTime).seconds
            "Dynamic" -> voltageCommand
            else -> 0.0
        }

    val drivetrain
        inline get() = test == "Drivetrain" || test == "Drivetrain (Angular)"
    fun reset() {
        SmartDashboard.putBoolean("SysIdOverflow", false)
        SmartDashboard.putString("SysIdTelemetry", "")
    }
}
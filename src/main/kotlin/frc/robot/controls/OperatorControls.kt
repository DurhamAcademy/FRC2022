package frc.robot.controls

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.button.Trigger

class OperatorControls {
    init {
        SmartDashboard.putNumber("shooterMult", 1.0)
        SmartDashboard.putNumber("back fudge", 100.0)
        SmartDashboard.putBoolean("compressor enabled", true)
    }
    val COMPRESSOR_TOGGLE = Trigger{SmartDashboard.getBoolean("compressor enabled", true)}
    // zero turret
    val COMPRESSOR_ENABLED = Trigger{SmartDashboard.getBoolean("compressor enabled", true)}
    // compressor toggle
    // disable subsystems
    // manual subsystems
}
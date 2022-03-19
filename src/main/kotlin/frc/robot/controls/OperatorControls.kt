package frc.robot.controls

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.kyberlib.pneumatics.KSolenoid
import frc.robot.commands.intake.ManualConveyor
import frc.robot.commands.intake.ManualIntake
import frc.robot.commands.shooter.ShooterCalibration
import frc.robot.commands.turret.ManualTurret

class OperatorControls {
    private val multString = "shooterMult"
    private val fudgeString = "back fudge"
    private val compressorString = "compressor enabled"

    init {
        SmartDashboard.putBoolean("nav updates", false)
        SmartDashboard.putNumber(multString, 1.0)
        SmartDashboard.putNumber(fudgeString, .06)
        SmartDashboard.putBoolean(compressorString, true)
        SmartDashboard.putBoolean("ManualTurret", false)
        SmartDashboard.putBoolean("ManualShooter", false)
        SmartDashboard.putBoolean("ManualConveyor", false)
        SmartDashboard.putBoolean("ManualIntake", false)
        SmartDashboard.putBoolean(compressorString, true)
    }

    val COMPRESSOR_ENABLED = Trigger { SmartDashboard.getBoolean(compressorString, true) }.whenActive {
        KSolenoid.compressor.enableDigital()
        println("enable compress")
        emptySet<Subsystem>()
    }.whenInactive {
        KSolenoid.compressor.disable()
        println("disable compress")
        emptySet<Subsystem>()
    }

    // disable subsystems
    // manual subsystems
    val MANUAL_TURRET = Trigger { SmartDashboard.getBoolean("ManualTurret", false) }.whileActiveOnce(ManualTurret)
    val MANUAL_SHOOTER =
        Trigger { SmartDashboard.getBoolean("ManualShooter", false) }.whileActiveOnce(ShooterCalibration)
    val MANUAL_CONVEYOR = Trigger { SmartDashboard.getBoolean("ManualConveyor", false) }.whileActiveOnce(ManualConveyor)
    val MANUAL_INTAKE = Trigger { SmartDashboard.getBoolean("ManualIntake", false) }.whileActiveOnce(ManualIntake)
}
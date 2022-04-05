package frc.robot.controls

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.kyberlib.command.Game
import frc.kyberlib.pneumatics.KSolenoid
import frc.robot.commands.climb.ManualClimb
import frc.robot.commands.climb.PrepareClimb
import frc.robot.commands.conveyor.ManualConveyor
import frc.robot.commands.intake.ManualIntake
import frc.robot.commands.shooter.ShooterCalibration
import frc.robot.commands.turret.ManualTurret

class OperatorControls {
    private val multString = "shooterMult"
    private val fudgeString = "back fudge"
    private val compressorString = "compressor enabled"

    val shootWhileMoving
        inline get() = SmartDashboard.getBoolean("move shot", false)
    val smartNav
        inline get() = SmartDashboard.getBoolean("nav updates", true)
    val curveComp
        inline get() = SmartDashboard.getNumber("curve comp", 0.0)
    val intakeCam
        inline get() = SmartDashboard.getBoolean("intake cam", false)
    val autoShot
        inline get() = SmartDashboard.getBoolean("auto shot", false)
    val climbStabilization
        inline get() = SmartDashboard.getNumber("stablizer", 0.0)

    init {
        SmartDashboard.putBoolean("nav updates", !Game.COMPETITION)
        SmartDashboard.putNumber(multString, 1.0)
        SmartDashboard.putNumber(fudgeString, .06)
        SmartDashboard.putBoolean(compressorString, true)

        SmartDashboard.putBoolean("invert drive motors", false)
        SmartDashboard.putBoolean("move shot", false)
        SmartDashboard.putBoolean("auto shot", false)
        SmartDashboard.putNumber("curve comp", -4.0)
        SmartDashboard.putNumber("stabalizer", 0.0)

        SmartDashboard.putBoolean("ManualTurret", false)
        SmartDashboard.putBoolean("ManualShooter", false)
        SmartDashboard.putBoolean("ManualConveyor", false)
        SmartDashboard.putBoolean("ManualIntake", false)
        SmartDashboard.putBoolean("ManualClimb", false)

        SmartDashboard.putBoolean("prepare for climb", false)
    }

    private val COMPRESSOR_ENABLED = Trigger { SmartDashboard.getBoolean(compressorString, true) }.whenActive {
        KSolenoid.compressor.enableDigital()
        println("enable compress")
        emptySet<Subsystem>()
    }.whenInactive {
        KSolenoid.compressor.disable()
        println("disable compress")
        emptySet<Subsystem>()
    }

    private val PREPARE =
        Trigger { SmartDashboard.getBoolean("prepare for climb", false) }.whileActiveOnce(PrepareClimb)

    // disable subsystems
    // manual subsystems
    private val MANUAL_TURRET =
        Trigger { SmartDashboard.getBoolean("ManualTurret", false) }.whileActiveOnce(ManualTurret)
    private val MANUAL_SHOOTER =
        Trigger { SmartDashboard.getBoolean("ManualShooter", false) }.whileActiveOnce(ShooterCalibration)
    private val MANUAL_CONVEYOR = Trigger { SmartDashboard.getBoolean("ManualConveyor", false) }.whileActiveOnce(
        ManualConveyor
    )
    private val MANUAL_INTAKE =
        Trigger { SmartDashboard.getBoolean("ManualIntake", false) }.whileActiveOnce(ManualIntake)
    private val MANUAL_CLIMB = Trigger { SmartDashboard.getBoolean("ManualClimb", false) }.whileActiveOnce(ManualClimb)
}
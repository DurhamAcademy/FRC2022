package frc.robot.commands.turret

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Turret

object ManualTurret: CommandBase() {
    private const val turretString = "Turret percent"
    init {
        addRequirements(Turret)
        SmartDashboard.putNumber(turretString, 0.1)
    }

    override fun execute() {
        Turret.percent = SmartDashboard.getNumber(turretString, 0.0)
    }
}
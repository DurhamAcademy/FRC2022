package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Turret

object LoadEject : CommandBase() {
    override fun initialize() {
        addRequirements(Shooter)
        addRequirements(Turret)
        addRequirements(Conveyor)
    }
    override fun execute() {

    }

    override fun isFinished(): Boolean {
        return super.isFinished()
    }
}
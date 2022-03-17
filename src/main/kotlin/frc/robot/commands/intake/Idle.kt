package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Conveyor

object Idle : CommandBase() {  // todo: think about Idle also spinning intake
    init {
        addRequirements(Conveyor)
    }

    override fun initialize() {
        Conveyor.idle()
    }
}
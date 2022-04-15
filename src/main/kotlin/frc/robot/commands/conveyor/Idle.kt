package frc.robot.commands.conveyor

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Conveyor

object Idle : CommandBase() {
    init { addRequirements(Conveyor) }
    override fun initialize() {
        Conveyor.idle()
    }
}
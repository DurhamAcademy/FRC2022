package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.rpm
import frc.robot.RobotContainer
import frc.robot.subsystems.ConveyorStatus
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.Shooter

object Feed : CommandBase() {
    init {
        addRequirements(Conveyor)
    }

    override fun initialize() {
        Conveyor.status = ConveyorStatus.FEEDING
        Conveyor.conveyor.percent = 0.8
        Conveyor.feeder.percent = 0.9
    }

    override fun isFinished(): Boolean = Shooter.stopped
}
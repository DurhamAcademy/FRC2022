package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.rotationsPerSecond
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.Shooter

object LoadEject : CommandBase() {
    override fun initialize() {
        addRequirements(Shooter)
        addRequirements(Conveyor)
    }

    override fun execute() {
        if (Shooter.flywheelMaster.velocity <= 7.rotationsPerSecond)
            Shooter.flywheelMaster.velocity = 7.rotationsPerSecond
        Conveyor.feeder.velocity = 1.rotationsPerSecond
        Conveyor.ConveyorMotor.velocity = 1.rotationsPerSecond
    }

    override fun isFinished(): Boolean {
        return (Shooter.flywheelMaster.velocity <= 6.5.rotationsPerSecond) &&
                (Conveyor.ConveyorMotor.velocity >= 0.25.rotationsPerSecond) &&
                Conveyor.feeder.velocity < 1.25.rotationsPerSecond
    }
}
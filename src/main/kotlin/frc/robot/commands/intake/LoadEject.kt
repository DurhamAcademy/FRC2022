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
        // MARK: Uncomment the stuff below if the ball is not getting into the shooter.
//        Conveyor.feeder.voltage = 0.9
//        Conveyor.ConveyorMotor.voltage = 0.1
    }

    override fun isFinished(): Boolean {
        return (Shooter.flywheelMaster.velocity <= 4.5.rotationsPerSecond)
    }
}
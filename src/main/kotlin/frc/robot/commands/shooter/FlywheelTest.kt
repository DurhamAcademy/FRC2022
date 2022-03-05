package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.Shooter

object FlywheelTest : CommandBase() {
    init {
        addRequirements(Shooter, Conveyor)
    }

    override fun initialize() {
        println("Shooter test")
        Conveyor.feeder.percent = 1.0
        Shooter.flywheelMaster.percent = .3
        Conveyor.conveyor.percent = 1.0
    }

    override fun end(interrupted: Boolean) {
        Shooter.flywheelMaster.stop()
        Conveyor.conveyor.stop()
    }

    override fun isFinished(): Boolean = false
}
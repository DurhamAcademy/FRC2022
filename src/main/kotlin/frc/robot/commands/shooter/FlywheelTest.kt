package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.centimeters
import frc.kyberlib.math.units.extensions.rpm
import frc.robot.RobotContainer
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.Shooter
import kotlin.math.absoluteValue

object FlywheelTest : CommandBase() {
    init {
        addRequirements(Shooter, Conveyor)
    }

    override fun execute() {
//        println("Shooter test")

        // 0 - 3000 rpm limits

        Shooter.targetVelocity = 2000.0.rpm
        Shooter.hoodDistance = 2.0.centimeters

        if(Shooter.hood.atSetpoint && (Shooter.targetVelocity.rpm - Shooter.flywheelMaster.velocity.rpm).absoluteValue < 100) {
            Conveyor.feeder.percent = 1.0
            Conveyor.conveyor.percent = 0.7
        } else {
            Conveyor.feeder.percent = -0.5
            Conveyor.conveyor.percent = 0.0
        }

    }

    override fun end(interrupted: Boolean) {
        Shooter.targetVelocity = 0.0.rpm
        Conveyor.conveyor.stop()
        Conveyor.feeder.percent = -0.2
        Shooter.hoodDistance = 0.0.centimeters
    }

    override fun isFinished(): Boolean = false
}
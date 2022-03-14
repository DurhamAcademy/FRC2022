package frc.robot.commands.shooter

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.*
import frc.robot.commands.intake.Feed
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.Shooter

object FlywheelTest : CommandBase() {
    init {
        addRequirements(Shooter)
        SmartDashboard.putNumber("flywheel rpm", 0.0)
        SmartDashboard.putNumber("hood degrees", 5.0)
    }

    override fun execute() {
        // 0 - 3000 rpm limits
//        Shooter.update()
        Shooter.targetVelocity = SmartDashboard.getNumber("flywheel rpm", 0.0).rpm
        Shooter.hoodDistance = SmartDashboard.getNumber("hood degrees", 0.0).millimeters

        if(Shooter.ready) {
            Feed.schedule()
        }

    }

    override fun end(interrupted: Boolean) {
        Shooter.stop()
    }

    override fun isFinished(): Boolean = false
}
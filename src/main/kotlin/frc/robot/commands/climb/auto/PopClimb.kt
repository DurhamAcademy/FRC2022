package frc.robot.commands.climb.auto

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.math.units.extensions.inches
import frc.robot.subsystems.Climber
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Turret

class PopClimb : CommandBase() {
    init {
        addRequirements(Climber, Drivetrain, Turret, Shooter)
    }

    override fun initialize() {
        Debug.log("Pop Climb", "init", level = DebugFilter.Low)
        Climber.extension = 30.inches

        Drivetrain.stop()
        Turret.turret.stop()
    }

    override fun execute() {
        Climber.updateMotors()
    }

    override fun end(interrupted: Boolean) {

    }

    override fun isFinished(): Boolean {
        return Climber.leftWinch.linearPositionError < 2.inches && Climber.leftWinch.linearPositionError < 2.inches
    }
}
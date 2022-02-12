package frc.robot.commands.climb.auto

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugLevel
import frc.kyberlib.math.units.extensions.degrees
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
        Debug.log("Pop Climb", "init", level = DebugLevel.LowPriority)
        Climber.extension = 30.inches
        Climber.extendableAngle = 85.degrees

        Drivetrain.stop()
        Turret.turret.stop()
    }

    override fun execute() {
        Climber.leftWinch.updateVoltage()
        Climber.rightWinch.updateVoltage()

        Climber.leftExtendable.updateVoltage()
        Climber.rightExtendable.updateVoltage()
    }

    override fun end(interrupted: Boolean) {
        Climber.extendableAngle = 90.degrees
    }

    override fun isFinished(): Boolean {
        return Climber.leftWinch.linearPositionError < 2.inches && Climber.leftWinch.linearPositionError < 2.inches
                && Climber.leftExtendable.positionError < 4.degrees && Climber.rightExtendable.positionError < 4.degrees
    }
}
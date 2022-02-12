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

class TraverseGrab : CommandBase() {
    init {
        addRequirements(Climber, Drivetrain, Turret, Shooter)
    }

    override fun initialize() {
        Debug.log("Traverse Grab", "init", level = DebugLevel.LowPriority)
        Climber.extendableAngle = 25.degrees
        Climber.extension = 30.inches
    }

    override fun execute() {
        Climber.updateMotors()
        if(Climber.leftWinch.linearPositionError < 2.inches) {
            Climber.extendableAngle = 45.degrees
        }
    }

    override fun end(interrupted: Boolean) {
        Climber.extension = 18.inches
    }

    override fun isFinished(): Boolean = false //Climber.leftExtendable.positionError < 3.degrees
}
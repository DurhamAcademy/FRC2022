package frc.robot.commands.climb.auto

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.math.units.extensions.inches
import frc.robot.subsystems.Climber
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Turret

class StaticLock : CommandBase() {
    init {
        addRequirements(Climber, Drivetrain, Turret, Shooter)
    }

    override fun initialize() {
        Debug.log("Static Lock", "init", level = DebugFilter.Low)
        Climber.extension = 0.inches
    }

    override fun execute() {
        Climber.updateMotors()
//        if (Climber.extension < 1.inches) {
//            Climber.staticsLifted = true
//            Climber.extension = 5.inches
//        } else if (Climber.extension < 20.inches) {
//            Climber.staticsLifted = false
//        }
    }

    override fun isFinished(): Boolean = Climber.leftWinch.linearPositionError < 2.inches && Climber.staticsLifted
}
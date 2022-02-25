package frc.robot.commands.climb.auto

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

val fullAutoClimb = SequentialCommandGroup(
    PopClimb(), StaticLock(), TraverseGrab(), StaticLock(), TraverseGrab()
)
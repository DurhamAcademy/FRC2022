package frc.kyberlib.auto

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.auto.trajectory.KTrajectory
import frc.kyberlib.mechanisms.drivetrain.KDrivetrain

class AutoDrive(drivetrain: KDrivetrain, val followerCommand: ()->Unit) : CommandBase() {
    constructor(drivetrain: KDrivetrain, trajectory: KTrajectory) : this(drivetrain, {drivetrain.drive(trajectory)})
    constructor(drivetrain: KDrivetrain, pose: Pose2d) : this(drivetrain, {drivetrain.driveTo(pose)})
    constructor(drivetrain: KDrivetrain, pos: Translation2d) : this(drivetrain, {drivetrain.driveTo(pos)})

    init { addRequirements(drivetrain) }

    override fun execute() {
        followerCommand.invoke()
    }
}
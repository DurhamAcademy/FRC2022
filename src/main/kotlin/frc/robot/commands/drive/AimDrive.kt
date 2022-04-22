package frc.robot.commands.drive

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.radians
import frc.robot.RobotContainer
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Limelight
import kotlin.math.absoluteValue

object AimDrive : CommandBase() {
    init {
        addRequirements(Drivetrain)
    }
    val corrector = PIDController(10.0, 0.0, 0.0)
    override fun execute() {
        val vx = 0.0
        val vy = 0.0
        val offset = if(Limelight.targetVisible) Limelight.visionOffset + Limelight.movementAngleOffset else Limelight.estimatedOffset
        val vO = corrector.calculate(-offset.radians)
        Drivetrain.drive(ChassisSpeeds(vx, vy, vO))
    }

    override fun isFinished(): Boolean = Limelight.visionOffset.absoluteValue < 2.degrees && Drivetrain.chassisSpeeds.omegaRadiansPerSecond.absoluteValue < 0.1
}
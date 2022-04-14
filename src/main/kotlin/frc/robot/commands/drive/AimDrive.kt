package frc.robot.commands.drive

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.radians
import frc.robot.RobotContainer
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Limelight

object AimDrive : CommandBase() {
    init {
        addRequirements(Drivetrain)
    }
    val corrector = PIDController(1.0, 0.0, 0.0)
    override fun execute() {
        var vx = 0.0
        var vy = 0.0
        if(RobotContainer.op.shootWhileMoving) {
            vx = RobotContainer.controlScheme.FORWARD
            vy = RobotContainer.controlScheme.STRAFE
        }

        val offset = if(Limelight.targetVisible) Limelight.visionOffset!! + Limelight.movementAngleOffset else Limelight.estimatedOffset
        val vO = corrector.calculate(offset.radians)
        Drivetrain.drive(ChassisSpeeds(vx, vy, vO))
    }
}
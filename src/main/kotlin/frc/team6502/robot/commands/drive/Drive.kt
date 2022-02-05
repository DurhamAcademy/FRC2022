package frc.team6502.robot.commands.drive

import edu.wpi.first.wpilibj.SlewRateLimiter
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.RobotContainer
import frc.team6502.robot.subsystems.Drivetrain
import kyberlib.math.units.extensions.feetPerSecond
import kyberlib.math.units.extensions.metersPerSecond
import kyberlib.math.units.extensions.radiansPerSecond

object Drive : CommandBase() {

    private val velFilter = SlewRateLimiter(10.0) // mps
    private val rotFilter = SlewRateLimiter(200.0)

    init {
        addRequirements(Drivetrain)
    }

    // TODO implement proper cheesy drive
    override fun execute() {
        val fwd = RobotContainer.controller.leftY.value.feetPerSecond
        val turn = RobotContainer.controller.rightX.value.radiansPerSecond

        println("drive execute")
        val speeds = ChassisSpeeds(velFilter.calculate(fwd.metersPerSecond), 0.0, rotFilter.calculate(turn.radiansPerSecond))
//        val speeds = DifferentialDriveWheelSpeeds(5.0, 5.0)
        Drivetrain.drive(speeds)
    }

    override fun isFinished() = false
}

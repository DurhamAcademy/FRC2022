package frc.robot.commands.shooter

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.*
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.Shooter
import kotlin.math.absoluteValue

object FlywheelTest : CommandBase() {
    init {
        addRequirements(Shooter, Conveyor)
//        SmartDashboard.putNumber("flywheel rpm", 0.0)
//        SmartDashboard.putNumber("hood degrees", 5.0)
    }

    override fun execute() {
//        println("Shooter test")

        // 0 - 3000 rpm limits
        val dis = SmartDashboard.getNumber("distance", 3.0)
        Shooter.targetVelocity = Constants.FLYWHEEL_INTERPOLATOR.calculate(dis)!!.rpm//SmartDashboard.getNumber("flywheel rpm", 0.0).rpm
        Shooter.hoodDistance = Constants.HOODANGLE_INTERPOLATOR.calculate(dis)!!.millimeters
//        Shooter.targetDistance?.meters?.let { SmartDashboard.putNumber("distance", it) }

        if(Shooter.hood.atSetpoint && (Shooter.flywheelMaster.velocityError.rpm).absoluteValue < 100 && Shooter.targetVelocity > 10.rpm) {
            Conveyor.feeder.percent = 1.0
            Conveyor.conveyor.percent = 0.7
        } else {
            Conveyor.feeder.percent = -0.0
            Conveyor.conveyor.percent = 0.0
        }

    }

    override fun end(interrupted: Boolean) {
        Shooter.targetVelocity = 0.0.rpm
        Conveyor.conveyor.stop()
        Conveyor.feeder.percent = -0.0
//        Shooter.hoodAngle = 0.degrees
    }

    override fun isFinished(): Boolean = false
}
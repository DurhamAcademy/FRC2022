package frc.robot.commands.intake

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.zeroPose
import frc.robot.RobotContainer
import frc.robot.commands.drive.AutoDrive
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.ConveyorStatus
import frc.robot.subsystems.Intaker

class AutoIntake : CommandBase() {
    init {
        addRequirements(Intaker)
    }

    var targetLocked = false
    var target: Pose2d = zeroPose
    val patience = Debouncer(0.1)

    override fun execute() {
        if(!RobotContainer.op.intakeCam) return
        if (targetLocked) {
            AutoDrive(target).schedule()
        } else {
            val result = RobotContainer.ballMonitor.latestResult
            if (result != null) {
                patience.calculate(true)
                target = RobotContainer.navigation.pose.transformBy(result.bestTarget.cameraToTarget)
            }else {
                patience.calculate(false)
            }
        }
    }

    override fun end(interrupted: Boolean) {
        if(!interrupted) {
            Conveyor.status = ConveyorStatus.SINGLE_GOOD
        }
    }

    override fun isFinished(): Boolean = !RobotContainer.op.intakeCam
}
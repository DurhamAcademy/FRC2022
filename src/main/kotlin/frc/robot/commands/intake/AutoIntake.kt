package frc.robot.commands.intake

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.zeroPose
import frc.robot.RobotContainer
import frc.robot.commands.drive.AutoDrive
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.ConveyorStatus
import frc.robot.subsystems.Intaker
import org.photonvision.PhotonUtils




class AutoIntake : CommandBase() {
    companion object {
        val cameraHeight = 10.inches
        val ballHeight = 10.5.inches
        val camPitch = 0.degrees
    }
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
//                if (result.hasTargets()) {
//                    val range = PhotonUtils.calculateDistanceToTargetMeters(
//                        cameraHeight.meters,
//                        ballHeight.meters / 2.0,
//                        camPitch.radians,
//                        (result.bestTarget.pitch).degrees.radians
//                    )
//
//                    val offset = result.bestTarget.yaw.degrees
//                    transform = Transform2d(Translation2d(range, offset.w), offset.w)
//
//                }
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
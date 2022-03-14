package frc.robot.commands.turret

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.TURRET_STATUS
import frc.robot.subsystems.Turret
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.robot.Constants

/**
 * Spin turret in circle. This command should never really be necessary if we odometry good
 */
object SeekTurret : CommandBase() {
    init {
        addRequirements(Turret)
    }
    // counts how long a target has been visible
    val acquisitionTimer = Timer()

    /**
     * Update status to indicated turret is lost
     */
    override fun initialize() {
        Turret.status = TURRET_STATUS.LOST
        timer.reset()
        timer.start()
    }

    private var direction = 1.0
    private val timer = Timer()
    override fun execute() {
        return

//        Turret.setTurretAngle(0.0)

//        if (Turret.targetVisible) {
//            if(timer.get() <= 0.0001)
//                timer.start()
//
//
//        } else {
//            timer.stop()
//            timer.reset()
//
//        }
//
//        SmartDashboard.putNumber("acq time", timer.get())
//
//        if(timer.get() > Constants.SHOOTER_AQUISITION_TIME) {
//            AimTurret.schedule()
//        }
//
//
//       /* Debug.log("Seek", "execute", level=DebugFilter.Low)
        if (Turret.targetVisible) { // the limelight sees something and it's valid if required
//             start timer if it isn't on yet
            if(acquisitionTimer.get() <= 0.0001) acquisitionTimer.start()
        } else {
            // the limelight doesn't see anything
            acquisitionTimer.stop()
            acquisitionTimer.reset()

            // sweeps menacingly
            Turret.turret.percent = 0.1 * direction
            if(Turret.turret.position < Turret.turret.minPosition!! + 5.degrees) direction = 1.0
            else if(Turret.turret.position > Turret.turret.maxPosition!! - 5.degrees) direction = -1.0
        }

        // if you are sure you see the target
        if(acquisitionTimer.get() > Constants.SHOOTER_AQUISITION_TIME) {
            // lock onto the target
            AimTurret.schedule()
        }//*/
    }

    override fun end(interrupted: Boolean) {
        if (!interrupted) Turret.status = TURRET_STATUS.ADJUSTING
        Turret.turret.stop()
    }
}
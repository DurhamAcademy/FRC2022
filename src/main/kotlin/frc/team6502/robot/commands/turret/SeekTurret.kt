package frc.team6502.robot.commands.turret

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.subsystems.Shooter
import frc.team6502.robot.subsystems.TURRET_STATUS
import frc.team6502.robot.subsystems.Turret
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.k

/**
 * Spin turret in circle. This command should never really be necessary if we odometry good
 */
object SeekTurret : CommandBase() {
    init {
        addRequirements(Turret)
    }
    // counts how long a target has been visible
    val acquisitionTimer = Timer()

    // todo move to Constants
    const val SHOOTER_AQUISITION_TIME = 0.2

    init {
        addRequirements(Shooter)
    }

    /**
     * Update status to indicated turret is lost
     */
    override fun initialize() {
        Turret.status = TURRET_STATUS.LOST
    }

    override fun execute() {
        if (!Turret.targetLost) { // the limelight sees something and it's valid if required
            // start timer if it isn't on yet
            if(acquisitionTimer.get() <= 0.0001) acquisitionTimer.start()
        } else {
            // the limelight doesn't see anything
            acquisitionTimer.stop()
            acquisitionTimer.reset()

            // spins menacingly
            Turret.fieldRelativeAngle = (Turret.fieldRelativeAngle + 5.degrees).k
        }

        // if you are sure you see the target
        if(acquisitionTimer.get() > SHOOTER_AQUISITION_TIME) {
            // lock onto the target
            AimTurret.schedule()
        }
    }

    override fun end(interrupted: Boolean) {
        if (!interrupted) Turret.status = TURRET_STATUS.ADJUSTING
    }
}
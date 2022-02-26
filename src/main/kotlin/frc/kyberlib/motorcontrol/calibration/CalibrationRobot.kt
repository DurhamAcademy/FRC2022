package frc.kyberlib.motorcontrol.calibration

import edu.wpi.first.wpilibj.TimedRobot
import frc.kyberlib.command.Game

class CalibrationRobot(val routine: CalibrationRoutine) : TimedRobot() {
    var prevTime = Game.time
    override fun autonomousPeriodic() {  // update motors
        val newTime = Game.time
        routine.update(newTime - prevTime)
        prevTime = newTime
    }

    override fun autonomousExit() {   // do math
        routine.doMath()
    }
}
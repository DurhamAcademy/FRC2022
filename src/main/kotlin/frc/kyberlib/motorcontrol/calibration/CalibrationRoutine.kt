package frc.kyberlib.motorcontrol.calibration

import frc.kyberlib.math.units.extensions.Time
import frc.kyberlib.motorcontrol.KMotorController


enum class CalibrationType {
    Drivetrain, Simple, Arm, Elevator
}

interface CalibrationRoutine {
    // make sure to use min / max angle
    // store output to file
    // figure out how recommended pid gains are calculated
    // figure out in general how sys id works - maybe add hooks into their code

    // volt
    // angle
    // velocity

    // ---- drivetrain ---- //
    // angle
    // angular rate
    fun update(dt: Time)

    fun doMath()

}

class DrivetrainCalibrationRoutine(val leftMaster: KMotorController, val rightMaster: KMotorController) : CalibrationRoutine {
    override fun update(dt: Time) {
        TODO("Not yet implemented")
    }

    override fun doMath() {
        TODO("Not yet implemented")
    }
}

// drivetrain / simple: ks * sign + kv * vel + ka * acc
// arm: kg * cos(angle) + ks * sign + kv * ang_vel + ka * ang_acc
// elevator: kg + ks * sign + kv * vel + ka * acc
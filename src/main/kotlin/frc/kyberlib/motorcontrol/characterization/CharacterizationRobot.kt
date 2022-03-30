package frc.kyberlib.motorcontrol.characterization

import edu.wpi.first.wpilibj.TimedRobot
import frc.kyberlib.motorcontrol.KMotorController

class CharacterizationRobot(vararg val routines: CharacterizationRoutine) : TimedRobot() {
    override fun autonomousInit() {
        // setup
        routines.forEach {
            it.initialize()
        }
    }

    override fun autonomousPeriodic() {
        // record data
        routines.forEach {
            it.execute()
        }
    }

    override fun autonomousExit() {
        // save
        routines.forEach {
            it.end(false)
        }
    }
}
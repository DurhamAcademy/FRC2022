package frc.kyberlib.tutorial.Tutorial2_Basic

import frc.kyberlib.command.KRobot
import frc.kyberlib.motorcontrol.rev.KSparkMax

class Robot : KRobot() {
    val motor = KSparkMax(0)
    override fun robotInit() {
        println("the robot started")
    }

    override fun robotPeriodic() {
        println("updating motors")
        motor.voltage = 6.0
    }
}
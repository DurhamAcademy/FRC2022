package frc.kyberlib.command

import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.motorcontrol.KBasicMotorController

class KSubsystem : Debug {
    companion object Static {
        var active: KSubsystem? = null
    }

    private inner class WpiSubsystem : SubsystemBase() {
        override fun periodic() {
            val time = Game.time
            this@KSubsystem.periodic()
            log()
        }

        override fun simulationPeriodic() {
            this@KSubsystem.simulationPeriodic()
        }
    }
    private val internal = WpiSubsystem()

    init {
        internal.name = identifier
        active = this
        Notifier {
            for (motor: KBasicMotorController in motors)
                if (motor.checkError()) {
                    manualControl = true
                    break
                }
            else manualControl = false
        }.startPeriodic(1.0)
    }

    private val motors = arrayListOf<KBasicMotorController>()
    private val solenoids = arrayListOf<Solenoid>()
    fun addMotor(motor: KBasicMotorController) {motors.add(motor)}
    fun addSolenoid(solenoid: Solenoid) {solenoids.add(solenoid)}
    private var manualControl = false

    open fun periodic() {

    }

    open fun simulationPeriodic() {

    }

    override fun debugValues(): Map<String, Any?> {
        val map = mutableMapOf<String, Any?>()
        motors.forEach { motor -> map[motor.identifier] = motor }
        solenoids.forEachIndexed { index, solenoid -> map["solenoid #$index"] = solenoid }
        return map.toMap()
    }
}
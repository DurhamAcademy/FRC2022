package frc.kyberlib.command

import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.motorcontrol.KBasicMotorController
import frc.kyberlib.pneumatics.KSolenoid


open class KSubsystem : Subsystem, Debug, Sendable {
    companion object {
        var active = "unset"
        var motorDump: MutableList<KBasicMotorController>? = null
        var solenoidDump: MutableList<KSolenoid>? = null
    }
    private val motors = arrayListOf<KBasicMotorController>()
    private val solenoids = arrayListOf<KSolenoid>()
    init {
        active = javaClass.simpleName
        motorDump = motors
        solenoidDump = solenoids
    }

    private inner class WpiSubsystem : SubsystemBase() {
        override fun periodic() {
            val time = Game.time
            for(motor in motors) motor.updateValues()
            if(!manualControl) this@KSubsystem.periodic()
            log("periodic time: ${Game.time - time}", level = DebugLevel.LowPriority)
        }

        override fun simulationPeriodic() {val time = Game.time
            if(!manualControl)
                this@KSubsystem.simulationPeriodic()
            debugDashboard("backend/")
            log("sim periodic time: ${Game.time - time}", level = DebugLevel.LowPriority)
        }
    }
    private val internal = WpiSubsystem()
    init {
        internal.name = javaClass.simpleName + "(internal)"
        Notifier {
            for (motor: KBasicMotorController in motors)
                if (motor.checkError()) {
                    manualControl = true
                    break
                }
            else manualControl = false
        }.startPeriodic(1.0)
    }

    private var manualControl = false
        set(value) {
            if (value && !field) {
                manual()
            }
            field = value
        }

    override fun periodic() {}
    override fun simulationPeriodic() {}

    override fun debugValues(): Map<String, Any?> {
        val map = mutableMapOf<String, Any?>()
        motors.forEach { motor -> map[motor.identifier] = motor }
        solenoids.forEachIndexed { index, solenoid -> map[solenoid.identifier] = solenoid }
        return map.toMap()
    }

    private fun manual() {
        motors.forEach {
            if(!it.checkError()) SmartDashboard.putData(it.identifier, it)
        }
        solenoids.forEach {
            SmartDashboard.putData(it.identifier, it)
        }
    }

    override fun initSendable(builder: SendableBuilder?) {
        internal.initSendable(builder)
    }
}
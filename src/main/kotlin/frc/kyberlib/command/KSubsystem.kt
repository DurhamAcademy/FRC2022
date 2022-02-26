package frc.kyberlib.command

import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Watchdog
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.motorcontrol.KBasicMotorController
import frc.kyberlib.pneumatics.KSolenoid


// todo: Check how command requirements work
open class KSubsystem : Subsystem, Debug, Sendable {

    override val priority: DebugLevel = DebugLevel.LowPriority

    companion object Static {
        var active: KSubsystem? = null
    }

    private inner class WpiSubsystem : SubsystemBase() {
        override fun periodic() {
            active = null
            val time = Game.time
            if(!manualControl)
                this@KSubsystem.periodic()
            log("periodic time: ${Game.time - time}")
        }

        override fun simulationPeriodic() {val time = Game.time
            if(!manualControl)
                this@KSubsystem.simulationPeriodic()
            debugDashboard("backend/")
            log("sim periodic time: ${Game.time - time}")
        }
    }
    private val internal = WpiSubsystem()

    init {
        internal.name = identifier
        run {active = this}
        Notifier {
            for (motor: KBasicMotorController in motors)
                if (motor.checkError()) {
                    manualControl = true
                    break
                }
            else manualControl = false
        }.startPeriodic(1.0)
    }

//    override fun getDefaultCommand(): Command? = internal.defaultCommand
//    override fun setDefaultCommand(value: Command?) { internal.defaultCommand = value }

    private val motors = arrayListOf<KBasicMotorController>()
    private val solenoids = arrayListOf<KSolenoid>()
    fun addMotor(motor: KBasicMotorController) {motors.add(motor)}
    fun addSolenoid(solenoid: KSolenoid) {solenoids.add(solenoid)}
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

        // todo: add default command
}
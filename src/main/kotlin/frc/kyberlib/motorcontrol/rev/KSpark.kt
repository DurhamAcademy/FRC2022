package frc.kyberlib.motorcontrol.rev

import edu.wpi.first.wpilibj.motorcontrol.Spark
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.command.LogMode
import frc.kyberlib.motorcontrol.BrakeMode
import frc.kyberlib.motorcontrol.KBasicMotorController

class KSpark(channel: Int) : KBasicMotorController() {
    val spark = Spark(channel)
    override var brakeMode: BrakeMode  = false  // this can't be set from code but by hitting button on spark you can change it
    override var identifier = "Spark$channel"
    override var rawPercent: Double = 0.0
        set(value) {
            field = value
            spark.set(value)
        }

    override fun followTarget(kmc: KBasicMotorController) {
        log("Enabling follow notifier. This can be slow", LogMode.WARN, DebugFilter.High)
        kmc.followers.add(this)
    }
}
package frc.kyberlib.motorcontrol.rev

import edu.wpi.first.wpilibj.motorcontrol.Spark
import frc.kyberlib.math.units.extensions.seconds
import frc.kyberlib.motorcontrol.BrakeMode
import frc.kyberlib.motorcontrol.KBasicMotorController

class KSpark(channel: Int) : KBasicMotorController() {
    private val spark = Spark(channel)
    override var brakeMode: BrakeMode  = false  // this can't be set from code but by hitting button on spark you can change it
    override var rawReversed: Boolean
        get() = spark.inverted
        set(value) {spark.inverted = value}
    override var identifier = "Spark$channel"
    override var rawPercent: Double = 0.0
        set(value) {
            field = value
            spark.set(value)
        }

    override fun followTarget(kmc: KBasicMotorController) {
        kmc.followers.add(this)
        kmc.notifier.startPeriodic(followPeriodic.seconds)
    }
}
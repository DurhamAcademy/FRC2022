package frc.team6502.robot.subsystems

import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team6502.robot.Constants
import kyberlib.command.Debug
import kyberlib.math.units.extensions.Angle
import kyberlib.math.units.extensions.Length
import kyberlib.math.units.extensions.degrees
import kyberlib.motorcontrol.rev.KSparkMax

object Shooter : SubsystemBase(), Debug {
    val flywheelMaster = KSparkMax(0).apply {
        identifier = "flywheel"
        // todo: make this not garbage
        kP = 1.0
        kD = 0.1

        radius = Constants.FLYWHEEL_RADIUS
    }
    private val flywheel2 = KSparkMax(0).apply { follow(flywheelMaster) }
    private val flywheel3 = KSparkMax(0).apply { follow(flywheelMaster) }
    private val flywheel4 = KSparkMax(0).apply { follow(flywheelMaster) }

    private val hood = Servo(1)

    // todo: figure out this will work - it won't figure it out
    var hoodAngle: Angle
        get() = hood.get().degrees
        set(value) {
            hood.set(value.degrees)
        }

    val targetLocked: Boolean = TODO()
    val targetDistance: Length = TODO()

    val topShooter = KSparkMax(0)

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "flywheel" to flywheelMaster,
            "top Shooter" to topShooter
        )
    }
}
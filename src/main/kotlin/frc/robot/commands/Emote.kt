package frc.robot.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.TAU
import frc.kyberlib.math.units.extensions.degrees
import frc.robot.subsystems.*
import kotlin.math.sin

object Emote : CommandBase() {
    private val timer = Timer()

    init {
        addRequirements(Drivetrain, Climber, Turret, Shooter, Intaker)
    }

    // how fast and much to turn
    private const val turnPeriod = 2.0
    private const val turnMag = 4.0

    // spinup emotes for emote
    private const val vroomPeriod = 4.0
    private const val vroomDuration = 0.3
    private const val vroomSpacing = 1.0
    private const val vroomIntensity = 0.3

    override fun initialize() {
        timer.reset()
        timer.start()
        Turret.turret.position = 0.degrees
        Intaker.deployed = true
    }

    override fun execute() {
        val t = timer.get()
        Drivetrain.drive(ChassisSpeeds(0.0, 0.0, sin(t * TAU / turnPeriod) * turnMag))

        Turret.turret.updateVoltage()
        if (Turret.turret.positionError.absoluteValue < 3.degrees) Climber.armsLifted = true

        val vroomTime = t.mod(vroomPeriod)
        if (vroomTime in 0.0..vroomDuration || vroomTime in vroomDuration + vroomSpacing..2 * vroomDuration + vroomSpacing)
            Shooter.flywheel.percent = vroomIntensity
        else Shooter.stop()
    }

    override fun end(interrupted: Boolean) {
        Intaker.deployed = false
        Climber.armsLifted = false
        Drivetrain.stop()
        Shooter.stop()
    }
}
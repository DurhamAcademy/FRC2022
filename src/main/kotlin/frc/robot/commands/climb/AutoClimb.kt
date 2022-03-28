package frc.robot.commands.climb

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.command.KRobot
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.math.units.extensions.seconds
import frc.robot.subsystems.*

object AutoClimb : CommandBase() {
    init {
        addRequirements(Climber, Drivetrain, Turret, Shooter)
    }

    /**
     * Prepare the climber to climb.
     */
    override fun initialize() {
        Debug.log("Climb Command", "init", level = DebugFilter.Low)
        Turret.turret.position = 0.degrees
        Climber.staticsLifted = true
        Climber.status = ClimberStatus.ACTIVE

        Climber.leftExtendable.position = 90.degrees
        Climber.rightExtendable.position = 90.degrees
    }

    var risen = false
    var driven = 0.5
    override fun execute() {
        if(!risen) {
            Climber.extension = 23.inches
            if(Climber.extension > 20.inches) risen = true
        } else if (driven > 0) {
            Climber.updateMotors()
            Drivetrain.drive(ChassisSpeeds(-.1, 0.0, 0.0))
            driven -= KRobot.period
        } else {
            Climber.extension = 0.inches
        }
    }

}
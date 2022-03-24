package frc.robot.commands.shooter

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.Translation2d
import frc.kyberlib.math.units.extensions.k
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.math.units.towards
import frc.kyberlib.math.units.zeroTranslation
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.subsystems.Conveyor
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Turret

/**
 * Takes the current balls
 */
object Dispose : CommandBase() {
    init {
        addRequirements(Shooter, Turret, Conveyor)
    }

    private val default: Translation2d = Translation2d(2.meters, Constants.FIELD_SIZE.y.meters - 2.meters)
    var disposalTarget: Translation2d = Translation2d(2.meters, Constants.FIELD_SIZE.y.meters - 2.meters)
    val timer = Timer()

    override fun initialize() {
        timer.reset()
        timer.start()
        val pos = RobotContainer.navigation.position
        val center = Constants.HUB_POSITION
        disposalTarget = if(pos.x > center.x && pos.y < center.y) default else zeroTranslation
    }

    override fun execute() {
        Turret.fieldRelativeAngle = RobotContainer.navigation.position.towards(disposalTarget).k
        val distance = RobotContainer.navigation.position.getDistance(disposalTarget).meters
        Shooter.flywheelUpdate(distance)
        Shooter.hoodUpdate(distance)
        if(timer.hasElapsed(0.3))
            Conveyor.feed()
    }
}
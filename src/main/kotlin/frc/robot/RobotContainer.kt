package frc.robot

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.kyberlib.auto.Navigator
import frc.kyberlib.auto.trajectory.TrajectoryManager
import frc.kyberlib.input.controller.KXboxController
import frc.kyberlib.sensors.gyros.KNavX
import frc.robot.commands.climb.Climb
import frc.robot.commands.intake.Intake
import frc.robot.commands.shooter.ForceShoot
import frc.robot.commands.shooter.Shoot
import frc.robot.subsystems.*

/**
 * Contains all Robot subsystems and sensors
 */
object RobotContainer {
    // initialize sensors and inputs here
    val gyro = KNavX()
    val limelight = frc.kyberlib.sensors.Limelight()

    // hall sensor
    val turretLimit = DigitalInput(0)

    val navigation = Navigator(gyro, Constants.START_POSE)

    // controllers
    val controller = KXboxController(0).apply {
        aButton.debounce(.3, Debouncer.DebounceType.kFalling).whileActiveOnce(Intake)
        rightTrigger.activateAt(.5).whenHeld(Shoot())
        rightBumper.whenHeld(ForceShoot)
        upDPad.toggleWhenPressed(Climb)
    }

    // auto path chooser
    val autoChooser = SendableChooser<String>().apply {
        val options = TrajectoryManager.routines
        for (path in options) addOption(path, path)
        setDefaultOption("Default", "Default")
        SmartDashboard.putData("auto", this)
    }

    init {
        // initialize subsystems here:
        Climber
        Conveyor
        Drivetrain
        Intaker
        Shooter
        Limelight
    }

}
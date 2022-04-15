package frc.robot.subsystems

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import frc.kyberlib.auto.Navigator
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.command.Game
import frc.kyberlib.math.PolarPose
import frc.kyberlib.math.polar
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.milli
import frc.kyberlib.mechanisms.drivetrain.DifferentialDriveTrain
import frc.kyberlib.mechanisms.drivetrain.dynamics.KDifferentialDriveDynamic
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.drive.Drive


/**
 * Mechanism that controls how the robot drives
 */
object Drivetrain : DifferentialDriveTrain() {
    // ff for each part of the drivetrain
    private val leftFF = SimpleMotorFeedforward(Constants.DRIVE_KS_L, Constants.DRIVE_KV_L, Constants.DRIVE_KA_L)
    private val rightFF = SimpleMotorFeedforward(Constants.DRIVE_KS_R, Constants.DRIVE_KV_R, Constants.DRIVE_KA_R)

    val leftMaster = KSparkMax(12).apply {
        identifier = "leftMaster"
        reversed = true
        brakeMode = true
        gearRatio = Constants.DRIVE_GEAR_RATIO
        radius = Constants.WHEEL_RADIUS
        currentLimit = 40

        kP = Constants.DRIVE_P
        addFeedforward(leftFF)
        motorType = DCMotor.getNEO(2)
        setupSim()
//        setupSim(leftFF)
    }
    val rightMaster = KSparkMax(13).apply {
        identifier = "rightMaster"
        copyConfig(leftMaster)
        addFeedforward(rightFF)
        setupSim()
//        setupSim(rightFF)
    }
    val leftFollower = KSparkMax(15).apply {
        copyConfig(leftMaster)
        follow(leftMaster)
    }
    val rightFollower = KSparkMax(10).apply {
        copyConfig(rightMaster)
        follow(rightMaster)  // follow(rightMaster)
    }

    override val dynamics: KDifferentialDriveDynamic = KDifferentialDriveDynamic(leftMaster, rightMaster)


    // setup motors
    init {
        defaultCommand = Drive
    }
}
package frc.robot.subsystems

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.Encoder
import frc.kyberlib.auto.Navigator
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.command.Game
import frc.kyberlib.math.PolarPose
import frc.kyberlib.math.polar
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.milli
import frc.kyberlib.mechanisms.drivetrain.DifferentialDriveTrain
import frc.kyberlib.mechanisms.drivetrain.dynamics.KDifferentialDriveDynamic
import frc.kyberlib.mechanisms.drivetrain.dynamics.KSwerveDynamics
import frc.kyberlib.mechanisms.drivetrain.swerve.StandardSwerveModule
import frc.kyberlib.mechanisms.drivetrain.swerve.SwerveDrive
import frc.kyberlib.motorcontrol.ctre.KTalon
import frc.kyberlib.motorcontrol.estimateFF
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.drive.Drive


/**
 * Mechanism that controls how the robot drives
 */
object Drivetrain : SwerveDrive() {
    // motors
    override val priority: DebugFilter = DebugFilter.Max

    // ff for each part of the drivetrain
    private val driveFF = SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA)
    private val turnFF = SimpleMotorFeedforward(Constants.TURN_KS, Constants.TURN_KV, Constants.TURN_KA)
    override val maxVelocity: LinearVelocity = 5.metersPerSecond//(12/driveFF.kv - 0.5).metersPerSecond

    val FLdrive = KTalon(0).apply {
        radius = 2.inches
        brakeMode = true
        gearRatio = Constants.DRIVE_GEAR_RATIO
        motorType = DCMotor.getFalcon500(1)
        addFeedforward(driveFF)  // todo
        kP = Constants.DRIVE_P
//        setupSim(flywheelSystem(driveMOI))
        setupSim()

    }
    val FLturn = KTalon(1).apply {
        kP = 3.0
        maxVelocity = 1.radiansPerSecond  // todo
        maxAcceleration = 1.radiansPerSecond
//        kD = 0.1
        motorType = DCMotor.getFalcon500(1)
        gearRatio = Constants.TURN_GEAR_RATIO
        brakeMode = true
        addFeedforward(turnFF)  // todo
        setupSim()
//        setupSim(flywheelSystem(turnMOI))
    }
    val FRdrive = KTalon(2).apply {
        copyConfig(FLdrive)
        setupSim()
//        setupSim(flywheelSystem(driveMOI))
    }
    val FRturn = KTalon(3).apply {
        copyConfig(FLturn)
        setupSim()
//        setupSim(flywheelSystem(turnMOI))
    }
    val BLdrive = KTalon(4).apply {
        copyConfig(FLdrive)
        setupSim()
//        setupSim(flywheelSystem(driveMOI))
    }
    val BLturn = KTalon(5).apply {
        copyConfig(FLturn)
        setupSim()
//        setupSim(flywheelSystem(turnMOI))
    }
    val BRdrive = KTalon(6).apply {
        copyConfig(FLdrive)
        setupSim()
//        setupSim(flywheelSystem(driveMOI))
    }
    val BRturn = KTalon(7).apply {
        copyConfig(FLturn)
        setupSim()
//        setupSim(flywheelSystem(turnMOI))
    }
    val frontLeft = StandardSwerveModule(Translation2d(-0.5, 0.5), FLdrive, FLturn)
    val frontRight = StandardSwerveModule(Translation2d(0.5, 0.5), FRdrive, FRturn)
    val backLeft = StandardSwerveModule(Translation2d(-0.5, -0.5), BLdrive, BLturn)
    val backRight = StandardSwerveModule(Translation2d(0.5, -0.5), BRdrive, BRturn)

    override val dynamics = KSwerveDynamics(frontLeft, frontRight, backLeft, backRight)

    // todo
    val flEncoder = Encoder(0, 0)
    val frEncoder = Encoder(0, 0)
    val blEncoder = Encoder(0, 0)
    val brEncoder = Encoder(0, 0)
    init {
        FLturn.resetPosition(flEncoder.distance.rotations)
        FRturn.resetPosition(frEncoder.distance.rotations)
        BLturn.resetPosition(blEncoder.distance.rotations)
        BRturn.resetPosition(brEncoder.distance.rotations)
    }


    // setup motors
    init {
        defaultCommand = Drive
        Navigator.instance!!.applyMovementRestrictions(3.metersPerSecond, 2.metersPerSecond)
    }

}
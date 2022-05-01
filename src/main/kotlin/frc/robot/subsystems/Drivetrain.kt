package frc.robot.subsystems

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.kyberlib.auto.Navigator
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.math.units.extensions.metersPerSecond
import frc.kyberlib.math.units.extensions.radiansPerSecond
import frc.kyberlib.mechanisms.drivetrain.HolonomicDrivetrain
import frc.kyberlib.mechanisms.drivetrain.dynamics.KSwerveDynamics
import frc.kyberlib.mechanisms.drivetrain.swerve.StandardSwerveModule
import frc.kyberlib.motorcontrol.ctre.KTalon
import frc.robot.Constants
import frc.robot.commands.drive.Drive


/**
 * Mechanism that controls how the robot drives
 */
object Drivetrain : HolonomicDrivetrain() {
    // motors
    override val priority: DebugFilter = DebugFilter.Max

    // ff for each part of the drivetrain
    private val driveFF = SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA)
    private val turnFF = SimpleMotorFeedforward(Constants.TURN_KS, Constants.TURN_KV, Constants.TURN_KA)

    val turnP = 10.0
    val FLdrive = KTalon(21).apply {
        radius = 2.inches
        brakeMode = true
        gearRatio = Constants.DRIVE_GEAR_RATIO
        motorType = DCMotor.getFalcon500(1)
        addFeedforward(driveFF)  // todo
//        kP = Constants.DRIVE_P
//        setupSim(flywheelSystem(driveMOI))
        setupSim()

    }
    val FLturn = KTalon(22).apply {
        maxVelocity = 1.radiansPerSecond  // todo
        maxAcceleration = 1.radiansPerSecond
//        kD = 0.1
        motorType = DCMotor.getFalcon500(1)
        gearRatio = Constants.TURN_GEAR_RATIO
        brakeMode = true
//        addFeedforward(turnFF)  // todo
//        nativeControl = true
//        talon.config_kP(0, 0.3)
        addFeedforward(turnFF)
        kP = turnP
        setupSim()
//        setupSim(flywheelSystem(turnMOI))
    }
    val FRdrive = KTalon(31).apply {
        radius = 2.inches
        brakeMode = true
        gearRatio = Constants.DRIVE_GEAR_RATIO
        motorType = DCMotor.getFalcon500(1)
        addFeedforward(driveFF)  // todo
//        kP = Constants.DRIVE_P
//        setupSim(flywheelSystem(driveMOI))
        setupSim()
//        setupSim(flywheelSystem(driveMOI))
    }
    val FRturn = KTalon(32).apply {
        copyConfig(FLturn)
        addFeedforward(turnFF)
        kP = turnP
//        nativeControl = true
//        talon.config_kP(0, 0.3)
        setupSim()
//        setupSim(flywheelSystem(turnMOI))
    }
    val BLdrive = KTalon(41).apply {
        copyConfig(FLdrive)
        radius = 2.inches
        brakeMode = true
        gearRatio = Constants.DRIVE_GEAR_RATIO
        motorType = DCMotor.getFalcon500(1)
        addFeedforward(driveFF)  // todo
//        kP = Constants.DRIVE_P
//        setupSim(flywheelSystem(driveMOI))
        setupSim()
//        setupSim(flywheelSystem(driveMOI))
    }
    val BLturn = KTalon(42).apply {
        copyConfig(FLturn)
        addFeedforward(turnFF)
        kP = turnP
//        nativeControl = true
//        talon.config_kP(0, 0.3)
        setupSim()
//        setupSim(flywheelSystem(turnMOI))
    }
    val BRdrive = KTalon(51).apply {
        radius = 2.inches
        brakeMode = true
        gearRatio = Constants.DRIVE_GEAR_RATIO
        motorType = DCMotor.getFalcon500(1)
        addFeedforward(driveFF)  // todo
//        kP = Constants.DRIVE_P
//        setupSim(flywheelSystem(driveMOI))
        setupSim()
//        setupSim(flywheelSystem(driveMOI))
    }
    val BRturn = KTalon(52).apply {
        copyConfig(FLturn)
        addFeedforward(turnFF)
        kP = turnP
//        nativeControl = true
//        talon.config_kP(0, 0.3)
        setupSim()
//        setupSim(flywheelSystem(turnMOI))
    }

    // todo - check positions
    val frontLeft = StandardSwerveModule(Translation2d(-0.3, 0.3), FLdrive, FLturn)
    val frontRight = StandardSwerveModule(Translation2d(0.3, 0.3), FRdrive, FRturn)
    val backLeft = StandardSwerveModule(Translation2d(-0.3, -0.3), BLdrive, BLturn)
    val backRight = StandardSwerveModule(Translation2d(0.3, -0.3), BRdrive, BRturn)

    override val dynamics = KSwerveDynamics(frontLeft, frontRight, backLeft, backRight)

    init {
        FLturn.resetPosition()
        FRturn.resetPosition()
        BLturn.resetPosition()
        BRturn.resetPosition()
    }

    fun robotDrive(chassisSpeeds: ChassisSpeeds) = dynamics.driveRobotRelative(chassisSpeeds)

    override fun periodic() {
        super.periodic()

        SmartDashboard.putString("chassis", chassisSpeeds.toString())
    }

    // setup motors
    init {
        defaultCommand = Drive
        Navigator.instance!!.applyMovementRestrictions(3.metersPerSecond, 2.metersPerSecond)
    }

}
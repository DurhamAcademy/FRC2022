package frc.robot.subsystems

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.kyberlib.auto.Navigator
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.mechanisms.drivetrain.HolonomicDrivetrain
import frc.kyberlib.mechanisms.drivetrain.dynamics.KSwerveDynamics
import frc.kyberlib.mechanisms.drivetrain.swerve.DifferentialModel
import frc.kyberlib.mechanisms.drivetrain.swerve.DifferentialSwerveModule
import frc.kyberlib.mechanisms.drivetrain.swerve.StandardSwerveModule
import frc.kyberlib.mechanisms.drivetrain.swerve.SwerveModule
import frc.kyberlib.motorcontrol.ctre.KTalon
import frc.kyberlib.simulation.Simulation
import frc.kyberlib.simulation.field.KField2d
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.drive.Drive
import kotlin.math.pow


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
        addFeedforward(driveFF)
    }
    val FLturn = KTalon(22).apply {
        motorType = DCMotor.getFalcon500(1)
        gearRatio = Constants.TURN_GEAR_RATIO
        brakeMode = true
        addFeedforward(turnFF)
        kP = turnP
    }
    val FRdrive = KTalon(31).apply {
        copyConfig(FLdrive)
        addFeedforward(driveFF)
    }
    val FRturn = KTalon(32).apply {
        copyConfig(FLturn)
        addFeedforward(turnFF)
    }
    val BLdrive = KTalon(41).apply {
        copyConfig(FLdrive)
        addFeedforward(driveFF)
    }
    val BLturn = KTalon(42).apply {
        copyConfig(FLturn)
        addFeedforward(turnFF)
    }
    val BRdrive = KTalon(51).apply {
        copyConfig(FLdrive)
        addFeedforward(driveFF)
    }
    val BRturn = KTalon(52).apply {
        copyConfig(FLturn)
        addFeedforward(turnFF)
    }

    // todo - check positions
    val model = DifferentialSwerveModule.model(DCMotor.getFalcon500(2), 0.00058, Constants.ROBOT_WEIGHT/4 * 2.inches.meters.pow(2), 1.0, 1.0)
    val gs = .1
    val gw = .1
    val otherModel = DifferentialSwerveModule.model(
        SimpleMotorFeedforward(0.0, 3.0, 0.2),
        SimpleMotorFeedforward(0.0, 3.0, 0.2), gs, gw)
    val activeModel = model
    val frontLeft = DifferentialSwerveModule(Translation2d(-0.3, 0.3), FLdrive, FLturn, gs, gw, 2.inches, activeModel)
    val frontRight = DifferentialSwerveModule(Translation2d(0.3, 0.3), FRdrive, FRturn, gs, gw, 2.inches, activeModel)
    val backLeft = DifferentialSwerveModule(Translation2d(-0.3, -0.3), BLdrive, BLturn, gs, gw, 2.inches, activeModel)
    val backRight = DifferentialSwerveModule(Translation2d(0.3, -0.3), BRdrive, BRturn, gs, gw, 2.inches, activeModel)

    override val dynamics = KSwerveDynamics(frontLeft, frontRight, backLeft, backRight)

    init {
        FLturn.resetPosition()
        FRturn.resetPosition()
        BLturn.resetPosition()
        BRturn.resetPosition()
        Simulation.include(frontLeft)
        Simulation.include(frontRight)
        Simulation.include(backLeft)
        Simulation.include(backRight)
    }

    fun robotDrive(chassisSpeeds: ChassisSpeeds) = dynamics.driveRobotRelative(chassisSpeeds)

    override fun periodic() {
        super.periodic()
        SmartDashboard.putNumber("top", frontLeft.topMotor.voltage)
        SmartDashboard.putNumber("bottom", frontLeft.bottomMotor.voltage)
        frontLeft.debug()
        SmartDashboard.putNumber("set speed", frontLeft.stateSetpoint.speedMetersPerSecond)
        SmartDashboard.putNumber("set angle", frontLeft.stateSetpoint.angle.radians)

        drawModules()
    }
    
    fun drawModules() {
        val poses = arrayOf(frontLeft, frontRight, backLeft, backRight).map {
            RobotContainer.navigation.pose.transformBy(Transform2d(it.location, it.rotation.w))
        }
        KField2d["wheel"].poses = poses
    }

    // setup motors
    init {
        defaultCommand = Drive
        Navigator.instance!!.applyMovementRestrictions(3.metersPerSecond, 2.metersPerSecond)
    }

}
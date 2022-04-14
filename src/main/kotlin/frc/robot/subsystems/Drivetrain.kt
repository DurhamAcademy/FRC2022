package frc.robot.subsystems

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
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
import frc.kyberlib.mechanisms.drivetrain.dynamics.KSwerveDynamics
import frc.kyberlib.mechanisms.drivetrain.swerve.StandardSwerveModule
import frc.kyberlib.mechanisms.drivetrain.swerve.SwerveDrive
import frc.kyberlib.motorcontrol.ctre.KTalon
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
    private val leftFF = SimpleMotorFeedforward(Constants.DRIVE_KS_L, Constants.DRIVE_KV_L, Constants.DRIVE_KA_L)
    private val rightFF = SimpleMotorFeedforward(Constants.DRIVE_KS_R, Constants.DRIVE_KV_R, Constants.DRIVE_KA_R)

    val FLdrive = KTalon(0).apply { setupSim() }
    val FLturn = KTalon(1).apply { setupSim() }
    val FRdrive = KTalon(2).apply { setupSim() }
    val FRturn = KTalon(3).apply { setupSim() }
    val BLdrive = KTalon(4).apply { setupSim() }
    val BLturn = KTalon(5).apply { setupSim() }
    val BRdrive = KTalon(6).apply { setupSim() }
    val BRturn = KTalon(7).apply { setupSim() }
    val frontLeft = StandardSwerveModule(Translation2d(-0.5, 0.5), FLdrive, FLturn)
    val frontRight = StandardSwerveModule(Translation2d(0.5, 0.5), FRdrive, FRturn)
    val backLeft = StandardSwerveModule(Translation2d(-0.5, -0.5), BLdrive, BLturn)
    val backRight = StandardSwerveModule(Translation2d(0.5, -0.5), BRdrive, BRturn)

    override val dynamics = KSwerveDynamics(frontLeft, frontRight, backLeft, backRight)

    /**
     * Get speeds relative to the hub.
     *
     * vx = speed towards the hub.
     *
     * vy = speed parallel to the hub.
     */
    val hubRelativeSpeeds: ChassisSpeeds
        inline get() {
            val polar = polarSpeeds
            return ChassisSpeeds(
                polar.dr.value,
                polar.dTheta.toTangentialVelocity(Limelight.distance).value,
                polar.dOrientation.radiansPerSecond
            )
        }
    var pose  // pose of the robot
        inline get() = RobotContainer.navigation.pose
        inline set(value) {
            val latency = RobotContainer.limelight.latency
            val detectionTime = Game.time - latency
            RobotContainer.navigation.update(value, detectionTime)
        }
    private var polarCoordinates  // polar coordinates relative to the Hub
        inline get() = RobotContainer.navigation.pose.polar(Constants.HUB_POSITION)
        inline set(value) {
            pose = Pose2d(value.cartesian(Constants.HUB_POSITION).translation, RobotContainer.navigation.heading.w)
        }
    val polarSpeeds
        inline get() = chassisSpeeds.polar(RobotContainer.navigation.pose.polar(Constants.HUB_POSITION))


    // setup motors
    init {
        defaultCommand = Drive
        Navigator.instance!!.applyMovementRestrictions(3.metersPerSecond, 2.metersPerSecond)
    }

    /**
     * Update navigation
     */
    override fun periodic() {
        super.periodic()
        debugDashboard()
        if (RobotContainer.op.smartNav && Game.OPERATED && Limelight.targetVisible) {
            // do global position updates based on limelight data
            val distance = Limelight.visionDistance ?: return
            val offset = Limelight.visionOffset ?: return
            val angle = offset + 180.degrees
            polarCoordinates = PolarPose(distance, angle, offset)
        }
    }

}
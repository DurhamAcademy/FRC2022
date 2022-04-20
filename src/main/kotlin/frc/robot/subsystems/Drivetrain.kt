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
    // motors
    override val priority: DebugFilter = DebugFilter.Max

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
        kI = Constants.DRIVE_I
        kD = Constants.DRIVE_D
        addFeedforward(leftFF)
        motorType = DCMotor.getNEO(2)
        setupSim()
//        setupSim(leftFF)
    }
    val rightMaster = KSparkMax(13).apply {
        identifier = "rightMaster"
        copyConfig(leftMaster)
        reversed = false
        addFeedforward(rightFF)
        setupSim()
//        setupSim(rightFF)
    }
    val leftFollower = KSparkMax(15).apply {
//        copyConfig(leftMaster)
        follow(leftMaster)
    }
    val rightFollower = KSparkMax(10).apply {
//        copyConfig(rightMaster)
        follow(rightMaster)  // follow(rightMaster)
    }

    override val dynamics: KDifferentialDriveDynamic = KDifferentialDriveDynamic(leftMaster, rightMaster)

    /**
     * Get speeds relative to the hub.
     *
     * vx = speed towards the hub.
     *
     * vy = speed parallel to the hub.
     */
    val hubRelativeSpeeds: ChassisSpeeds  // todo: see if there is faster calc
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
            val latency = RobotContainer.limelight.latestResult!!.latencyMillis.milli.seconds
            val detectionTime = Game.time - latency
            leftMaster.resetPosition()  // is this supposed to be resetting?
            rightMaster.resetPosition()
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
        if (RobotContainer.op.smartNav && Game.OPERATED && Turret.isZeroed) {
            // do global position updates based on limelight data
            val distance = Limelight.visionDistance ?: return
            val offset = Limelight.visionOffset ?: return
            val angle = offset + Turret.fieldRelativeAngle + 180.degrees
            polarCoordinates = PolarPose(distance, angle, offset)
        }
    }

}
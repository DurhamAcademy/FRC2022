package frc.robot.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.auto.Navigator
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.command.Game
import frc.kyberlib.math.PolarPose
import frc.kyberlib.math.polar
import frc.kyberlib.math.units.debugValues
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.milli
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.drive.Drive


/**
 * Mechanism that controls how the robot drives
 */
object Drivetrain : SubsystemBase(), Debug {
    // motors
    override val priority: DebugFilter = DebugFilter.Max

    // ff for each part of the drivetrain
    private val leftFF = SimpleMotorFeedforward(Constants.DRIVE_KS_L, Constants.DRIVE_KV_L, Constants.DRIVE_KA_L)
    private val rightFF = SimpleMotorFeedforward(Constants.DRIVE_KS_R, Constants.DRIVE_KV_R, Constants.DRIVE_KA_R)
    private val angularFeedforward = SimpleMotorFeedforward(0.6382, 2.7318, 0.32016)

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
        setupSim(leftFF)
    }
    val rightMaster = KSparkMax(13).apply {
        identifier = "rightMaster"
        copyConfig(leftMaster)
        addFeedforward(rightFF)
        setupSim(rightFF)
    }
    val leftFollower = KSparkMax(15).apply {
        copyConfig(leftMaster)
        follow(leftMaster)
    }
    val rightFollower = KSparkMax(10).apply {
        copyConfig(rightMaster)
        follow(rightMaster)  // follow(rightMaster)
    }


    val kinematics = DifferentialDriveKinematics(Constants.TRACK_WIDTH)  // calculator to make drivetrain move is the desired directions
    val chassisSpeeds: ChassisSpeeds  // variable representing the direction we want the robot to move
        inline get() = kinematics.toChassisSpeeds(wheelSpeeds)
    val wheelSpeeds: DifferentialDriveWheelSpeeds  // variable representing the speed of each side of the drivetrain
        inline get() = DifferentialDriveWheelSpeeds(
            leftMaster.linearVelocity.metersPerSecond,
            rightMaster.linearVelocity.metersPerSecond
        )

    /**
     * Speeds relative to the field
     */
    val fieldRelativeSpeeds: ChassisSpeeds
        inline get() {
            val chassis = chassisSpeeds
            val heading = RobotContainer.navigation.heading
            return ChassisSpeeds(
                chassis.vxMetersPerSecond * heading.cos,
                chassis.vxMetersPerSecond * heading.sin,
                chassis.omegaRadiansPerSecond
            )
        }

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
                polar.dTheta.toTangentialVelocity(Turret.targetDistance ?: RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION).meters).value,
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
        Navigator.instance!!.applyKinematics(kinematics)
    }

    /**
     * Drive the robot at the provided speeds
     */
    fun drive(speeds: ChassisSpeeds) {
        drive(kinematics.toWheelSpeeds(speeds))
    }

    /**
     * Drive the robot at the provided speeds
     */
    fun drive(wheelSpeeds: DifferentialDriveWheelSpeeds) {
        println(wheelSpeeds.leftMetersPerSecond)
        leftMaster.linearVelocity = wheelSpeeds.leftMetersPerSecond.metersPerSecond
        rightMaster.linearVelocity = wheelSpeeds.rightMetersPerSecond.metersPerSecond
    }

    private val anglePid = PIDController(0.5, 0.0, 0.0)
    fun fieldOrientedDrive(speed: LinearVelocity, direction: Angle) { // ignore this: I got bored on a flight
        val dTheta = RobotContainer.navigation.heading - direction + 90.degrees
        val vx = speed * dTheta.cos
        drive(ChassisSpeeds(vx.metersPerSecond, 0.0, anglePid.calculate(dTheta.radians)))
    }

    // stop the drivetrain
    fun stop() {
        leftMaster.stop()
        rightMaster.stop()
    }

    /**
     * Update navigation
     */
    override fun periodic() {
        RobotContainer.navigation.differentialDrive = true
        RobotContainer.navigation.update(
            wheelSpeeds, leftMaster.linearPosition, rightMaster.linearPosition
        )
        debugDashboard()

        if (RobotContainer.op.smartNav && Game.OPERATED && Turret.isZeroed) {
            // do global position updates based on limelight data
            val distance = Turret.targetDistance ?: return
            val offset = Turret.visionOffset ?: return
            val angle = offset + Turret.fieldRelativeAngle + 180.degrees
            polarCoordinates = PolarPose(distance, angle, offset)
        }
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "pose" to RobotContainer.navigation.pose.debugValues,
            "speed" to chassisSpeeds.debugValues,
            "leftMaster" to leftMaster,
            "rightMaster" to rightMaster
        )
    }

}
package frc.robot.subsystems

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.auto.Navigator
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.command.Game
import frc.kyberlib.math.PolarPose
import frc.kyberlib.math.filters.Differentiator
import frc.kyberlib.math.polar
import frc.kyberlib.math.units.debugValues
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.milli
import frc.kyberlib.mechanisms.drivetrain.KDrivetrain
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.simulation.Simulatable
import frc.kyberlib.simulation.Simulation
import frc.kyberlib.simulation.field.KField2d
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.drive.Drive


/**
 * Mechanism that controls how the robot drives
 */
object Drivetrain : SubsystemBase(), Debug, KDrivetrain, Simulatable {
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
    }
    val rightMaster = KSparkMax(13).apply {
        identifier = "rightMaster"
        reversed = false
        brakeMode = true
        gearRatio = Constants.DRIVE_GEAR_RATIO
        radius = Constants.WHEEL_RADIUS
        currentLimit = 40

        kP = Constants.DRIVE_P
        kI = Constants.DRIVE_I
        kD = Constants.DRIVE_D
        addFeedforward(rightFF)
        motorType = DCMotor.getNEO(2)
    }
    val leftFollower = KSparkMax(15).apply {
        identifier = "leftFollow"
        reversed = false
        brakeMode = true
        gearRatio = Constants.DRIVE_GEAR_RATIO
        radius = Constants.WHEEL_RADIUS

        kP = Constants.DRIVE_P
        kI = Constants.DRIVE_I
        kD = Constants.DRIVE_D
        addFeedforward(leftFF)
        currentLimit = 40
        motorType = DCMotor.getNEO(2)
        spark.follow(leftMaster.spark, reversed)  // follow(leftMaster)
    }
    val rightFollower = KSparkMax(10).apply {
        identifier = "rightFollow"
        currentLimit = 40
        brakeMode = true
        reversed = false
        gearRatio = Constants.DRIVE_GEAR_RATIO
        radius = Constants.WHEEL_RADIUS

        kP = Constants.DRIVE_P
        kI = Constants.DRIVE_I
        kD = Constants.DRIVE_D
        addFeedforward(rightFF)
        motorType = DCMotor.getNEO(2)
        spark.follow(rightMaster.spark, reversed)  // follow(rightMaster)
    }

    // whether to switch to using back motors
    val driveInversion
        get() = SmartDashboard.getBoolean("invert drive motors", false)


    val kinematics =
        DifferentialDriveKinematics(Constants.TRACK_WIDTH)  // calculator to make drivetrain move is the desired directions
    override var chassisSpeeds: ChassisSpeeds  // variable representing the direction we want the robot to move
        get() = kinematics.toChassisSpeeds(wheelSpeeds)
        set(value) {
            drive(value)
        }
    var wheelSpeeds: DifferentialDriveWheelSpeeds  // variable representing the speed of each side of the drivetrain
        get() = DifferentialDriveWheelSpeeds(
            leftMaster.linearVelocity.metersPerSecond,
            rightMaster.linearVelocity.metersPerSecond
        )
        set(value) {
            drive(value)
        }

    /**
     * Speeds relative to the field
     */
    val fieldRelativeSpeeds: ChassisSpeeds
        get() {
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
    val hubRelativeSpeeds: ChassisSpeeds
        get() {
            val polar = polarSpeeds
            return ChassisSpeeds(
                polar.dr.value,
                polar.dTheta.toTangentialVelocity(
                    Turret.targetDistance
                        ?: RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION).meters
                ).value,
                polar.dOrientation.radiansPerSecond
            )
        }
    var pose  // pose of the robot
        get() = RobotContainer.navigation.pose
        set(value) {
            val latency = RobotContainer.limelight.latestResult!!.latencyMillis.milli.seconds
            val detectionTime = Game.time - latency
            leftMaster.resetPosition()
            rightMaster.resetPosition()
            RobotContainer.navigation.update(value, detectionTime)
        }
    private var polarCoordinates  // polar coordinates relative to the Hub
        get() = RobotContainer.navigation.pose.polar(Constants.HUB_POSITION)
        set(value) {
            pose = Pose2d(value.cartesian(Constants.HUB_POSITION).translation, RobotContainer.navigation.heading.w)
        }
    val polarSpeeds
        get() = chassisSpeeds.polar(RobotContainer.navigation.pose.polar(Constants.HUB_POSITION))


    // setup motors
    init {
        defaultCommand = Drive
        Navigator.instance!!.applyMovementRestrictions(5.39.feetPerSecond, 2.metersPerSecond)
        Navigator.instance!!.applyKinematics(kinematics)
    }

    /**
     * Drive the robot at the provided speeds
     */
    override fun drive(speeds: ChassisSpeeds) {
        drive(kinematics.toWheelSpeeds(speeds))
    }

    /**
     * Drive the robot at the provided speeds
     */
    fun drive(wheelSpeeds: DifferentialDriveWheelSpeeds) {
        if (!driveInversion) {
            leftMaster.linearVelocity = wheelSpeeds.leftMetersPerSecond.metersPerSecond
            rightMaster.linearVelocity = wheelSpeeds.rightMetersPerSecond.metersPerSecond
        } else {
            leftFollower.voltage = wheelSpeeds.leftMetersPerSecond
            rightFollower.voltage = wheelSpeeds.rightMetersPerSecond
        }
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

    val dif = Differentiator()

    /**
     * Update navigation
     */
    override fun periodic() {
//        RobotContainer.navigation.update(
//            wheelSpeeds, leftMaster.linearPosition, rightMaster.linearPosition
//        )
        RobotContainer.navigation.poseEstimator.update(
            RobotContainer.gyro.heading.w,
            wheelSpeeds,
            leftMaster.linearPosition.meters,
            rightMaster.linearPosition.meters
        )
        SmartDashboard.putNumber("p spin", dif.calculate(RobotContainer.gyro.heading.radians))
        SmartDashboard.putNumber("c spin", chassisSpeeds.omegaRadiansPerSecond)
        SmartDashboard.putNumber("l_error", leftMaster.linearVelocityError.value)
        SmartDashboard.putNumber("r_error", rightMaster.linearVelocityError.value)

//        KField2d.robotPose = RobotContainer.navigation.pose
//        RobotContainer.navigation.update(wheelSpeeds, leftMaster.linearPosition, rightMaster.linearPosition)
        if (RobotContainer.op.smartNav && Game.OPERATED && Turret.isZeroed) {
            // do global position updates based on limelight data
            val distance = Turret.targetDistance ?: return
            val offset = Turret.visionOffset ?: return
            val angle = offset + Turret.fieldRelativeAngle + 180.degrees
            polarCoordinates = PolarPose(distance, angle, offset)
        }
    }

    override fun simulationPeriodic() {
        debugDashboard()
        KField2d.robotPose = RobotContainer.navigation.pose
    }

    // ignore this, it is sim and debug support
    private lateinit var driveSim: DifferentialDrivetrainSim
    private fun setupSim() {
        driveSim = DifferentialDrivetrainSim( // Create a linear system from our characterization gains.
            betterDrivetrainSystem(),
//            LinearSystemId.identifyDrivetrainSystem(leftFF.kv, leftFF.ka, angularFeedforward.kv, angularFeedforward.ka, Constants.TRACK_WIDTH),
            leftMaster.motorType!!,  // 2 NEO motors on each side of the drivetrain.
            leftMaster.gearRatio,  // gearing reduction
            kinematics.trackWidthMeters,  // The track width
            leftMaster.radius!!.meters,  // wheel radius
//             The standard deviations for measurement noise: x (m), y (m), heading (rad), L/R vel (m/s), L/R pos (m)
            VecBuilder.fill(0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001)
        )
        driveSim.pose = RobotContainer.navigation.pose
        if (Game.sim) Simulation.instance.include(this)
    }

    private fun betterDrivetrainSystem(): LinearSystem<N2, N2, N2> {
//        return LinearSystemId.identifyDrivetrainSystem(leftFF.kv, leftFF.ka, angularFeedforward.kv, angularFeedforward.ka)
        val kVAngular = angularFeedforward.kv// * 2.0 / Constants.TRACK_WIDTH
        val kAAngular = angularFeedforward.ka// * 2.0 / Constants.TRACK_WIDTH
        val A1: Double = 0.5 * -(leftFF.kv / leftFF.ka + kVAngular / kAAngular)
        val A2: Double = 0.5 * -(rightFF.kv / rightFF.ka - kVAngular / kAAngular)
        val B1: Double = 0.5 * (1.0 / leftFF.ka + 1.0 / kAAngular)
        val B2: Double = 0.5 * (1.0 / rightFF.ka - 1.0 / kAAngular)

        return LinearSystem(
            Matrix.mat(Nat.N2(), Nat.N2()).fill(A1, A2, A2, A1),
            Matrix.mat(Nat.N2(), Nat.N2()).fill(B1, B2, B2, B1),
            Matrix.mat(Nat.N2(), Nat.N2()).fill(1.0, 0.0, 0.0, 1.0),
            Matrix.mat(Nat.N2(), Nat.N2()).fill(0.0, 0.0, 0.0, 0.0)
        )
    }

    init {
        if (Game.sim) setupSim()
    }

    override fun simUpdate(dt: Time) {
        // update the sim with new inputs
        val leftVolt = leftMaster.voltage
        val rightVolt = rightMaster.voltage
        driveSim.setInputs(leftVolt, rightVolt)
        driveSim.update(dt.seconds)
//
//         update the motors with what they should be
        leftMaster.simLinearPosition = driveSim.leftPositionMeters.meters
        leftMaster.simLinearVelocity = driveSim.leftVelocityMetersPerSecond.metersPerSecond
        rightMaster.simLinearPosition = driveSim.rightPositionMeters.meters
        rightMaster.simLinearVelocity = driveSim.rightVelocityMetersPerSecond.metersPerSecond
        RobotContainer.gyro.heading = driveSim.heading.k
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
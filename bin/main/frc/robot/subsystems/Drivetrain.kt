package frc.robot.subsystems

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.math.VecBuilder
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.drive.Drive
import frc.robot.commands.shooter.Shoot
import frc.kyberlib.auto.Navigator
import frc.kyberlib.auto.trajectory.KTrajectory
import frc.kyberlib.auto.trajectory.KTrajectoryConfig
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.Translation2d
import frc.kyberlib.math.units.debugValues
import frc.kyberlib.math.units.extensions.feet
import frc.kyberlib.math.units.extensions.k
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.math.units.extensions.metersPerSecond
import frc.kyberlib.mechanisms.drivetrain.KDrivetrain
import frc.kyberlib.motorcontrol.MotorType
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.simulation.Simulatable
import frc.kyberlib.simulation.Simulation
import frc.kyberlib.simulation.field.KField2d
import frc.kyberlib.math.zeroIf
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin


/**
 * Mechanism that controls how the robot drives
 */
object Drivetrain : SubsystemBase(), KDrivetrain, Simulatable {
    // motors
    val leftMaster = KSparkMax(0, MotorType.BRUSHLESS).apply {
        identifier = "leftMaster"
        reversed = false
        currentLimit = 40
    }
    val rightMaster  = KSparkMax(0, MotorType.BRUSHLESS).apply {
        identifier = "rightMaster"
        reversed = true
        currentLimit = 40
    }
    private val leftFollower  = KSparkMax(0, MotorType.BRUSHLESS).apply {
        identifier = "leftFollow"
        currentLimit = 40
        follow(leftMaster)
    }
    private val rightFollower = KSparkMax(0, MotorType.BRUSHLESS).apply {
        identifier = "rightFollow"
        currentLimit = 40
        follow(rightMaster)
    }
    private val motors = arrayOf(leftMaster, rightMaster)

    // values
    val kinematics = DifferentialDriveKinematics(Constants.TRACK_WIDTH)  // calculator to make drivetrain move is the desired directions
    override var chassisSpeeds: ChassisSpeeds  // variable representing the direction we want the robot to move
        get() = kinematics.toChassisSpeeds(wheelSpeeds)
        set(value) { drive(value) }
    var wheelSpeeds: DifferentialDriveWheelSpeeds  // variable representing the speed of each side of the drivetrain
        get() = DifferentialDriveWheelSpeeds(leftMaster.linearVelocity.metersPerSecond, rightMaster.linearVelocity.metersPerSecond)
        set(value) { drive(value) }
    val fieldRelativeSpeeds: ChassisSpeeds  // robot speeds from the perspective of the driver
        get() {
            val pose = Navigator.instance!!.pose
            val robotRelativeSpeeds = chassisSpeeds
            val vx = chassisSpeeds.vxMetersPerSecond * cos(pose.rotation.radians)
            val vy = chassisSpeeds.vxMetersPerSecond * sin(pose.rotation.radians)
            return ChassisSpeeds(vx, vy, robotRelativeSpeeds.omegaRadiansPerSecond)
        }

    // commands
    /**
     * Drive the robot at the provided speeds
     */
    override fun drive(speeds: ChassisSpeeds) { drive(kinematics.toWheelSpeeds(speeds)) }

    /**
     * Drive the robot at the provided speeds
     */
    fun drive(wheelSpeeds: DifferentialDriveWheelSpeeds) {
        leftMaster.linearVelocity = wheelSpeeds.leftMetersPerSecond.metersPerSecond
        rightMaster.linearVelocity = wheelSpeeds.rightMetersPerSecond.metersPerSecond
    }

    fun stop() {
        leftMaster.stop()
        rightMaster.stop()
    }

    /**
     * Update navigation
     */
    override fun periodic() {
        debugDashboard()
        RobotContainer.navigation.update(wheelSpeeds)
        if(!Turret.targetLost && false)  {  // todo: test
            val distance = Shooter.targetDistance!! + 2.feet  // two feet is the radius of the hub
            val angle = Turret.visionOffset!! + Turret.fieldRelativeAngle
            val transform = Translation2d(distance, 0.meters).rotateBy(angle)
            val newPosition = Constants.HUB_POSITION.minus(transform)
            val time = Game.time - RobotContainer.limelight.latestResult!!.latencyMillis * 1000  // todo: wrong units
            RobotContainer.navigation.update(Pose2d(newPosition, RobotContainer.navigation.heading), time)

        }
    }

    override fun simulationPeriodic() {
        KField2d.robotPose = RobotContainer.navigation.pose
    }

    // setup motors
    init {
        defaultCommand = Drive

        // setup controls for drive motors
        val feedforward = SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA)
        if(Game.sim) Simulation.instance.chassisFF = feedforward
        for (motor in motors) {
            motor.apply {
                brakeMode = true
                gearRatio = Constants.DRIVE_GEAR_RATIO
                radius = Constants.WHEEL_RADIUS
                currentLimit = 40

                kP = Constants.DRIVE_P
                kI = Constants.DRIVE_I
                kD = Constants.DRIVE_D

                addFeedforward(feedforward)
            }
        }
        Navigator.instance!!.applyMovementRestrictions(7.metersPerSecond, 2.metersPerSecond)
        Navigator.instance!!.applyKinematics(kinematics)
    }


    // ignore this, it is sim and debug support
    private lateinit var driveSim: DifferentialDrivetrainSim
    fun setupSim(KvAngular: Double = 8.5, KaAngular: Double = 0.5) {
        driveSim = DifferentialDrivetrainSim( // Create a linear system from our characterization gains.
            LinearSystemId.identifyDrivetrainSystem(Constants.DRIVE_KV, Constants.DRIVE_KA, KvAngular, KaAngular),
            DCMotor.getNEO(2),  // 2 NEO motors on each side of the drivetrain.
            leftMaster.gearRatio,  // gearing reduction
            kinematics.trackWidthMeters,  // The track width
            leftMaster.radius!!.meters,  // wheel radius
            // The standard deviations for measurement noise: x (m), y (m), heading (rad), L/R vel (m/s), L/R pos (m)
            VecBuilder.fill(0.000, 0.000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000) // todo: add noise back
        )
    }

    private fun roundLows(v: Double): Double = if (v.absoluteValue < 0.2) 0.0 else v

    override fun simUpdate(dt: Double) {
        // update the sim with new inputs
        val leftVolt = leftMaster.voltage.zeroIf{it: Double -> it.absoluteValue < Constants.DRIVE_KS}
        val rightVolt = rightMaster.voltage.zeroIf{it: Double -> it.absoluteValue < Constants.DRIVE_KS}
        if (leftVolt == 0.0 && rightVolt == 0.0) return
        driveSim.setInputs(leftVolt, rightVolt)
        driveSim.update(dt)

        // update the motors with what they should be
        leftMaster.simLinearPosition = driveSim.leftPositionMeters.meters
        leftMaster.simLinearVelocity = driveSim.leftVelocityMetersPerSecond.metersPerSecond
        rightMaster.simLinearPosition = driveSim.rightPositionMeters.meters
        rightMaster.simLinearVelocity = driveSim.rightVelocityMetersPerSecond.metersPerSecond
        Navigator.instance!!.heading = driveSim.heading.k
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
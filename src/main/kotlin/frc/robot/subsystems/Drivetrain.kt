package frc.robot.subsystems

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.auto.Navigator
import frc.kyberlib.command.Game
import frc.kyberlib.math.PolarPose
import frc.kyberlib.math.invertIf
import frc.kyberlib.math.polar
import frc.kyberlib.math.units.debugValues
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.milli
import frc.kyberlib.mechanisms.drivetrain.KDrivetrain
import frc.kyberlib.motorcontrol.BrushType
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
object Drivetrain : SubsystemBase(), KDrivetrain, Simulatable {
    // motors
    val leftMaster = KSparkMax(10, BrushType.BRUSHLESS).apply {
        identifier = "leftMaster"
        reversed = false
        currentLimit = 40
        motorType = DCMotor.getNEO(2)
    }
    val rightMaster  = KSparkMax(12, BrushType.BRUSHLESS).apply {
        identifier = "rightMaster"
        reversed = true
        currentLimit = 40
        motorType = DCMotor.getNEO(2)
    }
    private val leftFollower  = KSparkMax(11, BrushType.BRUSHLESS).apply {
        identifier = "leftFollow"
        reversed = false
        currentLimit = 40
        follow(leftMaster)
    }
    private val rightFollower = KSparkMax(13, BrushType.BRUSHLESS).apply {
        identifier = "rightFollow"
        currentLimit = 40
        reversed = false
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
    var pose
        get() = RobotContainer.navigation.pose
        set(value) {
            val latency = RobotContainer.limelight.latestResult!!.latencyMillis.milli.seconds
            val detectionTime = Game.time - latency
            RobotContainer.navigation.update(value, detectionTime)
        }
    var polarCoordinates
        get() = RobotContainer.navigation.pose.polar(Constants.HUB_POSITION)
        set(value) {
            pose = Pose2d(value.cartesian(Constants.HUB_POSITION).translation, RobotContainer.navigation.heading)
        }
    val polarSpeeds
        get() = chassisSpeeds.polar(RobotContainer.navigation.pose.polar(Constants.HUB_POSITION))

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
        if(Turret.targetVisible && Constants.NAVIGATION_CORRECTION)  {  // TODO: test
            val distance = Shooter.targetDistance!!
            val angle = Turret.visionOffset!! + Turret.fieldRelativeAngle + 180.degrees
            polarCoordinates = PolarPose(distance, angle, Turret.visionOffset!!)
        }
    }

    override fun simulationPeriodic() {
        debugDashboard()
        KField2d.robotPose = RobotContainer.navigation.pose
    }

    // setup motors
    init {
        defaultCommand = Drive

        // setup controls for drive motors
        val feedforward = SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA)
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
        Navigator.instance!!.applyMovementRestrictions(5.39.feetPerSecond, 2.metersPerSecond)
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
//             The standard deviations for measurement noise: x (m), y (m), heading (rad), L/R vel (m/s), L/R pos (m)
            VecBuilder.fill(0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001)
        )
        driveSim.pose = RobotContainer.navigation.pose
        Simulation.instance.include(this)
    }

    init {
        if(Game.sim) setupSim()
    }
    override fun simUpdate(dt: Time) {
        // update the sim with new inputs
        val leftVolt = leftMaster.voltage.invertIf { leftMaster.reversed }//.zeroIf{ it.absoluteValue < Constants.DRIVE_KS}
        val rightVolt = rightMaster.voltage.invertIf { rightMaster.reversed }//.zeroIf{ it.absoluteValue < Constants.DRIVE_KS}
        if (leftVolt == 0.0 && rightVolt == 0.0) return
        driveSim.setInputs(leftVolt, rightVolt)
        driveSim.update(dt.seconds)
//
//         update the motors with what they should be
        leftMaster.simLinearPosition = driveSim.leftPositionMeters.meters
        leftMaster.simLinearVelocity = driveSim.leftVelocityMetersPerSecond.metersPerSecond
        rightMaster.simLinearPosition = driveSim.rightPositionMeters.meters
        rightMaster.simLinearVelocity = driveSim.rightVelocityMetersPerSecond.metersPerSecond
//        log("setpoint: $wheelSpeeds, sim: ${DifferentialDriveWheelSpeeds(driveSim.leftVelocityMetersPerSecond, driveSim.rightVelocityMetersPerSecond)}")
        Navigator.instance!!.heading = driveSim.heading.k
//        Navigator.instance!!.heading += chassisSpeeds.omegaRadiansPerSecond.radiansPerSecond * dt
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
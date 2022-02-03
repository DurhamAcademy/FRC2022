package kyberlib.mechanisms.drivetrain

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.system.plant.DCMotor
import edu.wpi.first.wpilibj.system.plant.LinearSystemId
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpiutil.math.VecBuilder
import kyberlib.command.Debug
import kyberlib.input.controller.KXboxController
import kyberlib.math.units.debugValues
import kyberlib.math.units.extensions.*
import kyberlib.motorcontrol.KMotorController
import kyberlib.sensors.gyros.KGyro
import kyberlib.simulation.Simulatable

/**
 * Stores important information for the motion of a DifferentialDrive Robot
 */
data class DifferentialDriveConfigs(val wheelRadius: Length, val trackWidth: Length)

/**
 * Pre-made DifferentialDrive Robot.
 * @param leftMotors array of all the motors on the left side of the robot. They will all follow the first
 * @param rightMotors array of all the motors on the right side of the robot. They will all follow the first
 * @param configs information about the physical desciption of this drivetrain
 * @param gyro KGyro to provide heading information
 */
class DifferentialDriveTrain(private val leftMotors: Array<KMotorController>, private val rightMotors: Array<KMotorController>,
                             private val configs: DifferentialDriveConfigs, val gyro: KGyro) : SubsystemBase(), Simulatable,
    KDrivetrain, Debug {
    constructor(leftMotor: KMotorController, rightMotor: KMotorController,
                configs: DifferentialDriveConfigs, gyro: KGyro) : this(arrayOf(leftMotor), arrayOf(rightMotor), configs, gyro)

    private val motors = leftMotors.plus(rightMotors)
    private val leftMaster = leftMotors[0]
    private val rightMaster = rightMotors[0]

    init {
        for (info in leftMotors.withIndex()) if (info.index > 0) info.value.follow(leftMaster)
        for (info in rightMotors.withIndex()) if (info.index > 0) info.value.follow(rightMaster)
        for (motor in motors)
            motor.radius = configs.wheelRadius
    }

    // control values
    private val odometry = DifferentialDriveOdometry(0.degrees)
    private val kinematics = DifferentialDriveKinematics(configs.trackWidth.meters)

    // useful information
    override var pose: Pose2d
        set(value) {
            odometry.resetPosition(value, gyro.heading)
        }
        get() = odometry.poseMeters
    override var heading
        get() = gyro.heading
        set(value) {gyro.heading = value}
    override val chassisSpeeds: ChassisSpeeds
        get() = kinematics.toChassisSpeeds(DifferentialDriveWheelSpeeds(leftMaster.linearVelocity.metersPerSecond, rightMaster.linearVelocity.metersPerSecond))

    // drive functions
    override fun drive(speeds: ChassisSpeeds) {
        drive(kinematics.toWheelSpeeds(speeds))
    }

    fun drive(speeds: DifferentialDriveWheelSpeeds) {
        leftMaster.linearVelocity = speeds.leftMetersPerSecond.metersPerSecond
        rightMaster.linearVelocity = speeds.rightMetersPerSecond.metersPerSecond
    }

    override fun periodic() {
        odometry.update(gyro.heading, leftMaster.linearPosition.meters, rightMaster.linearPosition.meters)
    }

    private lateinit var driveSim: DifferentialDrivetrainSim
    fun setupSim(KvLinear: Double, KaLinear: Double, KvAngular: Double, KaAngular: Double) {
        driveSim = DifferentialDrivetrainSim( // Create a linear system from our characterization gains.
            LinearSystemId.identifyDrivetrainSystem(KvLinear, KaLinear, KvAngular, KaAngular),
            DCMotor.getNEO(2),  // 2 NEO motors on each side of the drivetrain.
            1.0,  // gearing reduction
            1.0,  // The track width
            0.1,  // wheel radius
            // The standard deviations for measurement noise: x (m), y (m), heading (rad), L/R vel (m/s), L/R pos (m)
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
        )
    }

    override fun simUpdate(dt: Double) {
        // update the sim with new inputs
        driveSim.setInputs(leftMaster.voltage, rightMaster.voltage)
        driveSim.update(dt)

        // update the motors with what they should be
        leftMaster.simLinearPosition = driveSim.leftPositionMeters.meters
        leftMaster.simLinearVelocity = driveSim.leftVelocityMetersPerSecond.metersPerSecond
        rightMaster.simLinearPosition = driveSim.rightPositionMeters.meters
        rightMaster.simLinearVelocity = driveSim.rightVelocityMetersPerSecond.metersPerSecond
        gyro.heading = driveSim.heading.k
    }

    override fun debugValues(): Map<String, Any?> {
        val map = mutableMapOf(
            "pose" to pose.debugValues,
            "speed" to chassisSpeeds.debugValues,
            "leftMaster" to leftMaster,
            "rightMaster" to rightMaster
        )
        leftMotors.forEachIndexed { index, motor -> if (index != 0) map["leftFollow$index"] = motor }
        rightMotors.forEachIndexed { index, motor -> if (index != 0) map["rightFollow$index"] = motor }
        return map.toMap()
    }
}

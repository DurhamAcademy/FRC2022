package frc.kyberlib.mechanisms.drivetrain

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.math.VecBuilder
import frc.kyberlib.auto.Navigator
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.input.controller.KXboxController
import frc.kyberlib.math.units.debugValues
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.sensors.gyros.KGyro
import frc.kyberlib.simulation.Simulatable
import frc.kyberlib.simulation.Simulation

/**
 * Pre-made DifferentialDrive Robot.
 * @param leftMotors array of all the motors on the left side of the robot. They will all follow the first
 * @param rightMotors array of all the motors on the right side of the robot. They will all follow the first
 * @param configs information about the physical desciption of this drivetrain
 * @param gyro KGyro to provide heading information
 */
abstract class DifferentialDriveTrain: SubsystemBase(), Simulatable,
    KDrivetrain, Debug {

    abstract val leftMaster: KMotorController
    abstract val rightMaster: KMotorController
    abstract val trackWidth: Length

    // control values
    private val odometry = DifferentialDriveOdometry(0.degrees)
    private val kinematics = DifferentialDriveKinematics(trackWidth.meters)

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
        odometry.update(Navigator.instance!!.heading, leftMaster.linearPosition.meters, rightMaster.linearPosition.meters)
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
        Simulation.instance.include(this)
    }

    override fun simUpdate(dt: Time) {
        // update the sim with new inputs
        driveSim.setInputs(leftMaster.voltage, rightMaster.voltage)
        driveSim.update(dt.seconds)

        // update the motors with what they should be
        leftMaster.simLinearPosition = driveSim.leftPositionMeters.meters
        leftMaster.simLinearVelocity = driveSim.leftVelocityMetersPerSecond.metersPerSecond
        rightMaster.simLinearPosition = driveSim.rightPositionMeters.meters
        rightMaster.simLinearVelocity = driveSim.rightVelocityMetersPerSecond.metersPerSecond
        Navigator.instance!!.heading = driveSim.heading.k
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "pose" to Navigator.instance!!.pose.debugValues,
            "speed" to chassisSpeeds.debugValues,
            "leftMaster" to leftMaster,
            "rightMaster" to rightMaster
        )
    }
}

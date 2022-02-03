package frc.team6502.robot.SoftwareWorkshop

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj.system.plant.DCMotor
import edu.wpi.first.wpilibj.system.plant.LinearSystemId
import edu.wpi.first.wpiutil.math.VecBuilder
import frc.team6502.robot.Constants
import kyberlib.math.units.extensions.*
import kyberlib.motorcontrol.KMotorController
import kyberlib.sensors.gyros.KGyro
import kyberlib.simulation.Simulatable
import kyberlib.simulation.Simulation
import kyberlib.simulation.field.KField2d

abstract class SimDrivetrain : Simulatable {
    val trackWidth = 1.feet
    val wheelRadius = 2.inches
    val gearRatios = 1.0 / 20.0

    abstract val leftMaster: KMotorController
    abstract val rightMaster: KMotorController
    abstract val gyro: KGyro

    init {
        Simulation.instance.include(this)
    }

    private val driveSim = DifferentialDrivetrainSim( // Create a linear system from our characterization gains.
        LinearSystemId.identifyDrivetrainSystem(Constants.DRIVE_KV, Constants.DRIVE_KA, 1.5, 0.3),
        DCMotor.getNEO(2),  // 2 NEO motors on each side of the drivetrain.
        gearRatios,  // gearing reduction
        trackWidth.meters,  // The track width
        wheelRadius.meters,  // wheel radius
        // The standard deviations for measurement noise: x (m), y (m), heading (rad), L/R vel (m/s), L/R pos (m)
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
    )

    val odometry = DifferentialDriveOdometry(0.degrees)

    final override fun simUpdate(dt: Double) {
        // update the sim with new inputs
        driveSim.setInputs(leftMaster.voltage, rightMaster.voltage)
        driveSim.update(dt)

        // update the motors with what they should be - no setting this in your code
        leftMaster.simLinearPosition = driveSim.leftPositionMeters.meters
        leftMaster.simLinearVelocity = driveSim.leftVelocityMetersPerSecond.metersPerSecond
        rightMaster.simLinearPosition = driveSim.rightPositionMeters.meters
        rightMaster.simLinearVelocity = driveSim.rightVelocityMetersPerSecond.metersPerSecond
        gyro.heading = driveSim.heading.k

        odometry.update(driveSim.heading, driveSim.leftPositionMeters, driveSim.rightPositionMeters)
        KField2d.robotPose = odometry.poseMeters
    }
}
package frc.kyberlib.mechanisms.drivetrain

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.auto.Navigator
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.command.KRobot
import frc.kyberlib.math.units.debugValues
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.simulation.Simulatable
import frc.kyberlib.simulation.Simulation

/**
 * Pre-made DifferentialDrive Robot.
 * @param leftMotors array of all the motors on the left side of the robot. They will all follow the first
 * @param rightMotors array of all the motors on the right side of the robot. They will all follow the first
 * @param configs information about the physical desciption of this drivetrain
 * @param gyro KGyro to provide heading information
 */
abstract class DifferentialDriveTrain : SubsystemBase(), Simulatable, KDrivetrain, Debug {

    abstract val leftMaster: KMotorController
    abstract val rightMaster: KMotorController
    abstract val trackWidth: Length

    abstract val leftFF: SimpleMotorFeedforward
    abstract val rightFF: SimpleMotorFeedforward
    abstract val angularFeedforward: SimpleMotorFeedforward

    // control values
    private val kinematics = DifferentialDriveKinematics(trackWidth.meters)

    override val chassisSpeeds: ChassisSpeeds
        get() = kinematics.toChassisSpeeds(wheelSpeeds)
    val wheelSpeeds
        get() = DifferentialDriveWheelSpeeds(
            leftMaster.linearVelocity.metersPerSecond,
            rightMaster.linearVelocity.metersPerSecond
        )

    // drive functions
    override fun drive(speeds: ChassisSpeeds) {
        drive(kinematics.toWheelSpeeds(speeds))
    }

    fun drive(speeds: DifferentialDriveWheelSpeeds) {
        leftMaster.linearVelocity = speeds.leftMetersPerSecond.metersPerSecond
        rightMaster.linearVelocity = speeds.rightMetersPerSecond.metersPerSecond
    }

    override fun periodic() {
        Navigator.instance!!.update(wheelSpeeds, leftMaster.linearPosition, rightMaster.linearPosition)
    }

    init {
        if (Game.sim) setupSim()
    }

    private lateinit var driveSim: DifferentialDrivetrainSim
    fun setupSim() {
        driveSim = DifferentialDrivetrainSim( // Create a linear system from our characterization gains.
            driveSystem,
            DCMotor.getNEO(2),  // 2 NEO motors on each side of the drivetrain.
            leftMaster.gearRatio,  // gearing reduction
            trackWidth.value,  // The track width
            leftMaster.radius!!.value,  // wheel radius
            // The standard deviations for measurement noise: x (m), y (m), heading (rad), L/R vel (m/s), L/R pos (m)
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
        )
        if (Game.sim) Simulation.include(this)
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
//        RobotContainer = driveSim.heading.k
    }

    private val driveSystem: LinearSystem<N2, N2, N2>
        get() {
            val kVAngular = angularFeedforward.kv * 2.0 / trackWidth.value
            val kAAngular = angularFeedforward.ka * 2.0 / trackWidth.value
            val A1: Double = 0.5 * -(leftFF.kv / kAAngular + kVAngular / kAAngular)
            val A2: Double = 0.5 * -(rightFF.kv / kAAngular + kVAngular / kAAngular)
            val B1: Double = 0.5 * (1.0 / leftFF.ka + 1.0 / kAAngular)
            val B2: Double = 0.5 * (1.0 / rightFF.ka - 1.0 / kAAngular)

            return LinearSystem(
                Matrix.mat(Nat.N2(), Nat.N2()).fill(A1, A2, A2, A1),
                Matrix.mat(Nat.N2(), Nat.N2()).fill(B1, B2, B2, B1),
                Matrix.mat(Nat.N2(), Nat.N2()).fill(1.0, 0.0, 0.0, 1.0),
                Matrix.mat(Nat.N2(), Nat.N2()).fill(0.0, 0.0, 0.0, 0.0)
            )
        }
    private val velocityTolerance = 0.5
    private val optimizer = LinearQuadraticRegulator(
        driveSystem,
        VecBuilder.fill(velocityTolerance, velocityTolerance),  // left/right velocity error tolerance
        VecBuilder.fill(Game.batteryVoltage, Game.batteryVoltage),
        KRobot.period
    )
    private val observer = KalmanFilter(
        N2.instance, N2.instance,
        driveSystem,
        VecBuilder.fill(3.0, 3.0),
        VecBuilder.fill(.01, 0.01),
        KRobot.period
    )
    private val loop = LinearSystemLoop(driveSystem, optimizer, observer, Game.batteryVoltage, KRobot.period)


    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "pose" to Navigator.instance!!.pose.debugValues,
            "speed" to chassisSpeeds.debugValues,
            "leftMaster" to leftMaster,
            "rightMaster" to rightMaster
        )
    }
}

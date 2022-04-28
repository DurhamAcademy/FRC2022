package frc.kyberlib.mechanisms.drivetrain.dynamics

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import frc.kyberlib.TRACK_WIDTH
import frc.kyberlib.auto.Navigator
import frc.kyberlib.auto.trajectory.KTrajectory
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.motorcontrol.Voltage
import frc.kyberlib.motorcontrol.characterization.CharacterizationRoutine

// https://github.com/Team254/FRC-2020-Public/blob/master/src/main/java/com/team254/lib/physics/DifferentialDrive.java

/**
 * Class to handle the driving a differential drive chassis
 *
 * Features to add:
 * drag compensation better
 * characterization mode builtin to get moi and mass
 * better turn
 * statespace
 */
class KDifferentialDriveDynamic(val leftMaster: KMotorController, val rightMaster: KMotorController,
                                val angularFF: SimpleMotorFeedforward= SimpleMotorFeedforward(0.0, 0.0), val angularDrag: Double = 0.0) : KDriveDynamics() {

    companion object {
        fun toWheelSpeeds(chassisSpeeds: ChassisSpeeds, trackWidth: Length= TRACK_WIDTH): DifferentialDriveWheelSpeeds {
            return DifferentialDriveWheelSpeeds(
                chassisSpeeds.vxMetersPerSecond - trackWidth.meters / 2 * chassisSpeeds.omegaRadiansPerSecond,
                chassisSpeeds.vxMetersPerSecond + trackWidth.meters / 2 * chassisSpeeds.omegaRadiansPerSecond)
        }

        fun toChassisSpeed(wheelSpeeds: DifferentialDriveWheelSpeeds, trackWidth: Length = TRACK_WIDTH): ChassisSpeeds {
            return ChassisSpeeds(
                (wheelSpeeds.leftMetersPerSecond + wheelSpeeds.rightMetersPerSecond) / 2,
                0.0,
                (wheelSpeeds.rightMetersPerSecond - wheelSpeeds.leftMetersPerSecond) / trackWidth.meters)
        }
    }

    inline val wheelSpeeds
        get() = DifferentialDriveWheelSpeeds(leftMaster.linearVelocity.metersPerSecond, rightMaster.linearVelocity.metersPerSecond)
    override val chassisSpeeds: ChassisSpeeds
        get() {
            val averageSpeed = (leftMaster.linearVelocity.value + rightMaster.linearVelocity.value) / 2.0
            val fwd = averageSpeed
            val turn = averageSpeed * TRACK_WIDTH.meters
            return ChassisSpeeds(fwd, 0.0,  turn)
        }

    val curvature
        get() = chassisSpeeds.omegaRadiansPerSecond / chassisSpeeds.vxMetersPerSecond

    val kinematics = DifferentialDriveKinematics(TRACK_WIDTH.meters)
    init {
        Navigator.instance!!.applyKinematics(kinematics)
        Navigator.instance!!.differentialDrive = true
        instance = this
    }

    /**
     * Drive chassis speeds
     * @param chassisSpeeds the chassis speeds object representing how to drive the robot
     */
    override fun drive(chassisSpeeds: ChassisSpeeds) {
        val left = chassisSpeeds.vxMetersPerSecond - TRACK_WIDTH.value * chassisSpeeds.omegaRadiansPerSecond
        val right = chassisSpeeds.vxMetersPerSecond + TRACK_WIDTH.value * chassisSpeeds.omegaRadiansPerSecond
        drive(left.metersPerSecond, right.metersPerSecond)
    }

    /**
     * Drive a pair of wheel speeds
     * @param wheelSpeeds object containing how fast we side of the drivetrain should drive
     */
    fun drive(wheelSpeeds: DifferentialDriveWheelSpeeds) {
        drive(wheelSpeeds.leftMetersPerSecond.metersPerSecond, wheelSpeeds.rightMetersPerSecond.metersPerSecond)
    }

    /**
     * Set each side of the drivetrain to drive at teh specified speed
     */
    @JvmName("field relative drive1")
    fun drive(leftVelocity: LinearVelocity, rightVelocity: LinearVelocity) {
        leftMaster.linearVelocity = leftVelocity
        rightMaster.linearVelocity = rightVelocity
    }

    /**
     * Voltage drive the robot
     */
    fun drive(leftVoltage: Voltage, rightVoltage: Voltage) {
        leftMaster.voltage = leftVoltage
        rightMaster.voltage = rightVoltage
    }

    val rotationPID = PIDController(0.5, 0.0, 0.0)

    /**
     * skuffed field oriented drive. This was just a random idea and its not really useful
     */
    @JvmName("field relative drive")
    fun drive(speed: LinearVelocity, direction: Angle) {
        val dTheta = Navigator.instance!!.heading - direction + 90.degrees
        val vx = speed * dTheta.cos
        drive(vx, rotationPID.calculate(dTheta.radians).radiansPerSecond)
    }

    // auto driving
    private var startTime = Game.time
    private val controller = RamseteController()

    /**
     * Drive along a trajectory
     */
    override fun drive(path: KTrajectory) {
        if(activeTrajectory == null || path != activeTrajectory) {
            activeTrajectory = path
            startTime = Game.time
        }
        val dt = Game.time - startTime
        if (dt > activeTrajectory!!.totalTimeSeconds.seconds)
            drive(controller.calculate(Navigator.instance!!.pose, activeTrajectory!!.sample(dt.seconds)))
    }

    /**
     * Stop the drivetrain
     */
    override fun stop() {
        leftMaster.stop()
        rightMaster.stop()
    }

    private fun stateSpace(leftFF: SimpleMotorFeedforward, rightFF: SimpleMotorFeedforward, angularFeedforward: SimpleMotorFeedforward): LinearSystem<N2, N2, N2> {
        val kVAngular = angularFeedforward.kv * 2.0 / TRACK_WIDTH.meters
        val kAAngular = angularFeedforward.ka * 2.0 / TRACK_WIDTH.meters
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
    
    val characterizationRoutine
        get() = CharacterizationRoutine(leftMaster, rightMaster, drivetrain = true)
}
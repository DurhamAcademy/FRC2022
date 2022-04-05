package frc.kyberlib.mechanisms

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.LinearSystemLoop
import frc.kyberlib.auto.Navigator
import frc.kyberlib.auto.pathing.Pathfinder
import frc.kyberlib.auto.trajectory.KTrajectory
import frc.kyberlib.command.Game
import frc.kyberlib.command.KRobot
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.string
import frc.kyberlib.math.units.towards
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.motorcontrol.Voltage
import org.ietf.jgss.GSSManager

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
class KDifferentialDriveDynamic(val leftMaster: KMotorController, val rightMaster: KMotorController, val trackWidth: Length,
                                val angularFF: SimpleMotorFeedforward= SimpleMotorFeedforward(0.0, 0.0), val angularDrag: Double = 0.0) {
//    private val wheelRadius = leftMaster.radius!!

    var wheelSpeeds
        get() = KDifferentialDriveWheelSpeeds(leftMaster.linearVelocity, rightMaster.linearVelocity)
        set(value) = drive(value)
    var chassisSpeeds: KChassisSpeeds
        get() {
            val averageSpeed = (leftMaster.linearVelocity.value + rightMaster.linearVelocity.value) / 2.0
            val fwd = averageSpeed
            val turn = averageSpeed * trackWidth.meters
            return KChassisSpeeds(fwd.metersPerSecond,  turn.radiansPerSecond)
        }
        set(value) = drive(value)

    val wheelAcceleration: KDifferentialDriveWheelSpeeds
        get() = KDifferentialDriveWheelSpeeds(leftMaster.linearAcceleration, rightMaster.linearAcceleration)
    val chassisAcceleration: KChassisSpeeds
        get() {
            val averageSpeed = (leftMaster.linearAcceleration.value + rightMaster.linearAcceleration.value) / 2.0
            val fwd = averageSpeed
            val turn = averageSpeed * trackWidth.meters
            return KChassisSpeeds(fwd.metersPerSecond, turn.radiansPerSecond)
        }

    val curvature
        get() = chassisSpeeds.turn.value / chassisSpeeds.fwd.value

    /**
     * Drive speeds
     * @param fwd how fast the robot should drive fwd
     * @param turn how fast the robot should try to turn
     */
    @JvmName("drive1")
    fun drive(fwd: LinearVelocity, turn: AngularVelocity) {
        val left = fwd.value - trackWidth.value * turn.value
        val right = fwd.value + trackWidth.value * turn.value
        drive(left.metersPerSecond, right.metersPerSecond)
    }

    /**
     * Drive chassis speeds
     * @param chassisSpeeds the chassis speeds object representing how to drive the robot
     */
    fun drive(chassisSpeeds: KChassisSpeeds) {
        drive(chassisSpeeds.fwd, chassisSpeeds.turn)
    }

    /**
     * Drive a pair of wheel speeds
     * @param wheelSpeeds object containing how fast we side of the drivetrain should drive
     */
    fun drive(wheelSpeeds: KDifferentialDriveWheelSpeeds) {
        drive(wheelSpeeds.left, wheelSpeeds.right)
    }

    /**
     * Set each side of the drivetrain to drive at teh specified speed
     */
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
    var activeTrajectory: KTrajectory? = null
    private var startTime = Game.time
    private val controller = RamseteController()

    /**
     * Drive along a trajectory
     */
    fun drive(trajectory: KTrajectory) {
        if(activeTrajectory == null || trajectory != activeTrajectory) {
            activeTrajectory = trajectory
            startTime = Game.time
        }
        val dt = Game.time - startTime
        if (dt > trajectory.totalTimeSeconds.seconds)
            drive(controller.calculate(Navigator.instance!!.pose, trajectory.sample(dt.seconds)).k)
    }

    /**
     * Drive to a pose
     * @param goal the pose to drive to
     * @param direct whether to drive straight to goal or do pathplanning for obstacle avoidance
     */
    fun driveTo(goal: Pose2d, direct: Boolean = true) {
        if(activeTrajectory != null && activeTrajectory!!.states.last().poseMeters != goal) {
            val currentLocation = Navigator.instance!!.pose
            if (direct)
                KTrajectory(goal.string, currentLocation, emptyList(), goal)
            else {
                activeTrajectory = Pathfinder.pathTo(currentLocation, goal)
            }
        }
        activeTrajectory
        drive(activeTrajectory!!)
    }
    /**
     * Drive to a pose
     * @param goal the position to drive to
     * @param direct whether to drive straight to goal or do pathplanning for obstacle avoidance
     */
    fun driveTo(goal: Translation2d, direct: Boolean = true) {
        if(activeTrajectory != null && activeTrajectory!!.states.last().poseMeters.translation != goal) {
            val currentLocation = Navigator.instance!!.pose
            if (direct)
                KTrajectory(goal.toString(), currentLocation, emptyList(), Pose2d(goal, currentLocation.translation.towards(goal)))
            else {
                activeTrajectory = Pathfinder.pathTo(currentLocation, goal)
            }
        }
            activeTrajectory
        drive(activeTrajectory!!)
    }
    /**
     * remove the active trajectory data
     */
    fun clearTrajectory() {activeTrajectory = null}

    /**
     * Update the Navigator class with the current wheel motion
     */
    fun updateNavigation() {
        Navigator.instance!!.update(wheelSpeeds.w, leftMaster.linearPosition, rightMaster.linearPosition)
    }

    /**
     * Stop the drivetrain
     */
    fun stop() {
        leftMaster.stop()
        rightMaster.stop()
    }

    data class KChassisSpeeds(val fwd: LinearVelocity, val turn: AngularVelocity) {
        constructor(chassisSpeeds: ChassisSpeeds) : this(chassisSpeeds.vxMetersPerSecond.metersPerSecond, chassisSpeeds.omegaRadiansPerSecond.radiansPerSecond)
        val w
            get() = ChassisSpeeds(fwd.metersPerSecond, 0.0, turn.radiansPerSecond)
    }
    data class KDifferentialDriveWheelSpeeds(val left: LinearVelocity, val right: LinearVelocity) {
        constructor(speeds: DifferentialDriveWheelSpeeds) : this(speeds.leftMetersPerSecond.metersPerSecond, speeds.rightMetersPerSecond.metersPerSecond)
        val w
            get() = DifferentialDriveWheelSpeeds(left.metersPerSecond, right.metersPerSecond)
    }

    private fun stateSpace(leftFF: SimpleMotorFeedforward, rightFF: SimpleMotorFeedforward, angularFeedforward: SimpleMotorFeedforward): LinearSystem<N2, N2, N2> {
        val kVAngular = angularFeedforward.kv * 2.0 / trackWidth.meters
        val kAAngular = angularFeedforward.ka * 2.0 / trackWidth.meters
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
        if(Game.sim) setupSim()
    }
    private fun setupSim() {

    }

    init {
        val placeholder = SimpleMotorFeedforward(0.0, 0.0)
        val driveSystem = stateSpace(placeholder, placeholder, placeholder)
        val loop = LinearSystemLoop(
            driveSystem,
            LinearQuadraticRegulator(
                driveSystem,
                VecBuilder.fill(0.1, 0.1),  // left/right velocity error tolerance
                VecBuilder.fill(Game.batteryVoltage, Game.batteryVoltage),
                KRobot.period
            ),
            KalmanFilter(
                N2.instance, N2.instance,
                driveSystem,
                VecBuilder.fill(3.0, 3.0),
                VecBuilder.fill(.01, 0.01),
                KRobot.period
            ),
            Game.batteryVoltage,
            KRobot.period
        )


    }
}

val ChassisSpeeds.k
    get() = KDifferentialDriveDynamic.KChassisSpeeds(this.vxMetersPerSecond.metersPerSecond, this.omegaRadiansPerSecond.radiansPerSecond)

val DifferentialDriveWheelSpeeds.k
    get() = KDifferentialDriveDynamic.KDifferentialDriveWheelSpeeds(leftMetersPerSecond.metersPerSecond, rightMetersPerSecond.metersPerSecond)
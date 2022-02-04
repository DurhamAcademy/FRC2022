package frc.team6502.robot.subsystems

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj.system.plant.DCMotor
import edu.wpi.first.wpilibj.system.plant.LinearSystemId
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpiutil.math.VecBuilder
import frc.team6502.robot.Constants
import frc.team6502.robot.RobotContainer
import kyberlib.auto.Navigator
import kyberlib.command.Debug
import kyberlib.command.Game
import kyberlib.math.units.debugValues
import kyberlib.math.units.extensions.k
import kyberlib.math.units.extensions.meters
import kyberlib.math.units.extensions.metersPerSecond
import kyberlib.motorcontrol.MotorType
import kyberlib.motorcontrol.rev.KSparkMax
import kyberlib.simulation.Simulatable
import kyberlib.simulation.Simulation
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin


object Drivetrain : SubsystemBase(), Simulatable, Debug {
    // motors
    private val leftMaster = KSparkMax(0, MotorType.BRUSHLESS).apply {
        identifier = "leftMaster"
        reversed = false
        currentLimit = 40
    }
    private val rightMaster  = KSparkMax(0, MotorType.BRUSHLESS).apply {
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

    // setup
    init {
        // add drive motors to debug
        leftMaster.debugDashboard()
        rightMaster.debugDashboard()

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
        if (Game.sim) Simulation.instance.chassisFF = feedforward
    }

    // values
    val kinematics = DifferentialDriveKinematics(Constants.TRACK_WIDTH)
    var chassisSpeeds: ChassisSpeeds
        get() = kinematics.toChassisSpeeds(wheelSpeeds)
        set(value) { drive(value) }
    var wheelSpeeds: DifferentialDriveWheelSpeeds
        get() = DifferentialDriveWheelSpeeds(leftMaster.linearVelocity.metersPerSecond, rightMaster.linearVelocity.metersPerSecond)
        set(value) { drive(value) }
    val fieldRelativeSpeeds: ChassisSpeeds
        get() {
            val pose = Navigator.instance!!.pose
            val robotRelativeSpeeds = chassisSpeeds
            val vx = chassisSpeeds.vxMetersPerSecond * cos(pose.rotation.radians)
            val vy = chassisSpeeds.vxMetersPerSecond * sin(pose.rotation.radians)
            return ChassisSpeeds(vx, vy, robotRelativeSpeeds.omegaRadiansPerSecond)
        }

    // commands - todo
    fun drive(chassisSpeeds: ChassisSpeeds) { drive(kinematics.toWheelSpeeds(chassisSpeeds)) }
    fun drive(differentialDriveWheelSpeeds: DifferentialDriveWheelSpeeds) {}
    fun stop() {
        leftMaster.stop()
        rightMaster.stop()
    }


    // ignore this, it is sim and debug support
    private lateinit var driveSim: DifferentialDrivetrainSim
    fun setupSim(KvAngular: Double = 5.5, KaAngular: Double = 0.5) {
        driveSim = DifferentialDrivetrainSim( // Create a linear system from our characterization gains.
            LinearSystemId.identifyDrivetrainSystem(Simulation.instance.chassisFF.kv, Simulation.instance.chassisFF.ka, KvAngular, KaAngular),
            DCMotor.getNEO(2),  // 2 NEO motors on each side of the drivetrain.
            leftMaster.gearRatio,  // gearing reduction
            kinematics.trackWidthMeters,  // The track width
            leftMaster.radius!!.meters,  // wheel radius
            // The standard deviations for measurement noise: x (m), y (m), heading (rad), L/R vel (m/s), L/R pos (m)
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
        )
    }

    private fun roundLows(d: Double): Double = if (d.absoluteValue < 0.2) 0.0 else d

    override fun simUpdate(dt: Double) {
        // update the sim with new inputs
        val leftVolt = leftMaster.voltage
        val rightVolt = rightMaster.voltage
        driveSim.setInputs(roundLows(leftVolt), roundLows(rightVolt))
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
            "rightMaster" to rightMaster,
        )
    }

}
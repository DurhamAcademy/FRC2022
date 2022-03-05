package frc.robot.subsystems

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.motorcontrol.KSimulatedESC
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.simulation.Simulatable
import frc.kyberlib.simulation.Simulation
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.subsystems.Turret.targetVisible


/**
 * Current status of the shooter mechanism
 */
enum class ShooterStatus {
    IDLE, SPINUP, SHOT, FORCE_SHOT
}


/**
 * Encapsulates all the things relevant to shooting the ball
 */
object Shooter : SubsystemBase(), Debug, Simulatable {
    var status = ShooterStatus.IDLE
    val ff = SimpleMotorFeedforward(0.17708, 0.020178, 0.00061369)

    // main motor attached to the flywheel
    val flywheelMaster = KSimulatedESC(31).apply {
        identifier = "flywheel"
//        radius = Constants.FLYWHEEL_RADIUS
        motorType = DCMotor.getNEO(2)
        kP = 0.0002
        addFeedforward(ff)

//        val system = flywheelSystem(Constants.FLYWHEEL_MOMENT_OF_INERTIA)
//        val loop = KMotorController.StateSpace.systemLoop(system, 3.0, 0.01, 3.0)
//        setupSim(system)
//        if(Constants.doStateSpace)
//            stateSpaceControl(loop)
    }

    var outputVelocity
        get() = flywheelMaster.velocity.toTangentialVelocity(Constants.BALL_DIAMETER - Constants.SHOOTER_COMPRESSION)
        set(value) {
            flywheelMaster.velocity = value.toAngularVelocity(Constants.BALL_DIAMETER - Constants.SHOOTER_COMPRESSION)
        }
    var timeOfFlight = 1.seconds


    // additional motors that copy the main
    private val flywheel2 = KSimulatedESC(32).apply {
        identifier = "flywheel2"
        reversed = true
        follow(flywheelMaster)
    }

    // Servo that sets the hood angle
    val hood = KSimulatedESC(1).apply {
        identifier = "hood"
    }

    var hoodAngle: Angle
        get() = hood.position
        set(value) {
            hood.position = value
        }

    // how far from the center of the hub the robot is (based on limelight)
    private val distanceFilter = LinearFilter.movingAverage(8)
    val targetDistance: Length? 
        get() = if (Game.sim) RobotContainer.navigation.position.getDistance(Constants.HUB_POSITION).meters
                else if (!targetVisible) null
                else 2.feet + distanceFilter.calculate((
                (Constants.UPPER_HUB_HEIGHT - 1.inches - Constants.LIMELIGHT_HEIGHT) / (Constants.LIMELIGHT_ANGLE + Turret.visionPitch!!).tan).inches).inches

    // motor controlling top roller speed
    val topShooter = KSimulatedESC(33).apply {
        identifier = "top1"
        kP = 10.0
        kD = 2.0
        motorType = DCMotor.getNeo550(2)
        val system = flywheelSystem(0.00001)
        if (Constants.doStateSpace) stateSpaceControl(system, 3.0, 0.01, 8.0)
        setupSim(system)
    }
    private val topFollower = KSimulatedESC(34).apply {
        identifier = "top2"
        follow(topShooter)
        reversed = true
    }

    init {
        if(Game.sim) Simulation.instance.include(this)
    }

    override fun periodic() {
        debugDashboard()
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "flywheel" to flywheelMaster,
            "top Shooter" to topShooter,
            "hood" to hoodAngle,
            "distance" to targetDistance
        )
    }

    override fun simUpdate(dt: Time) {
//        flywheelControl.simUpdate(dt)
        hood.simPosition = hood.positionSetpoint
//        topShooter.simVelocity = topShooter.velocitySetpoint
    }
}
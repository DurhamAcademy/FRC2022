package frc.robot.subsystems

import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.math.filters.Differentiator
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.pneumatics.KSolenoid
import frc.kyberlib.simulation.Simulatable
import frc.kyberlib.simulation.Simulation
import frc.robot.Constants
import frc.robot.RobotContainer
import kotlin.math.absoluteValue


/**
 * Modes for what the climber is doing. Useful for LEDs and other dependencies
 */
enum class ClimberStatus {
    IDLE, ACTIVE, RISING, FALLING
}

/**
 * Mechanism representing the actuators for climbing
 */
object Climber : SubsystemBase(), Debug, Simulatable {
    var status = ClimberStatus.IDLE

    // pneumatics that lift the climb arms
    private val leftStatic = KSolenoid(7, 6, fake = false).apply {
        identifier = "left static"
    }
    private val rightStatic = KSolenoid(0, 1, fake = false).apply {
        identifier = "right static"
    }

    /** (left) winches that pull the robot up */
    val leftWinch = KSparkMax(40).apply {
        identifier = "left winch"
        radius = Constants.WINCH_RADIUS
        brakeMode = true
        gearRatio = Constants.WINCH_GEAR_RATIO
        // state space for closed loop control     mass(kg)                                     model std       measure std     pos tolerance   vel tolerance
//        stateSpaceControl(elevatorSystem(120 * MassConversions.poundsToGrams * 1000.0), 5.inches.value, 3.inches.value, 1.inches.value, (3.inches/1.seconds).value, 12.0)
        minLinearPosition = 0.inches
        maxLinearPosition = 30.inches
        motorType = DCMotor.getNEO(1)
        currentLimit = 30
//        if(Game.sim) setupSim(winchFF)
    }

    /** (right) winches that pull the robot up */
    val rightWinch = KSparkMax(41).apply {
        identifier = "right winch"
        radius = Constants.WINCH_RADIUS
        brakeMode = true
        gearRatio = Constants.WINCH_GEAR_RATIO
        customControl = { bangBang(it) }
        minLinearPosition = 0.inches
        maxLinearPosition = 30.inches
        motorType = DCMotor.getNEO(1)
        currentLimit = 30
    }

    /** public variable to get/set whether the arms are lifted */
    var staticsLifted
        get() = leftStatic.extended
        set(value) {
            leftStatic.extended = !value
            rightStatic.extended = !value
        }
    var extension  // public variable to control position off both the arms
        get() = leftWinch.linearPosition
        set(value) {
            leftWinch.linearPosition = value
            rightWinch.linearPosition = value
        }

    // update closed-loop controls
    fun updateMotors() {
        leftWinch.updateVoltage()
        rightWinch.updateVoltage()
    }

    /**
     * Bang bang control used winch position control.
     * If too low, go up hard.
     * If too high, go down hard.
     * @return voltage represented as double
     */
    fun bangBang(motor: KMotorController): Double {
        return if (motor.positionError.degrees.absoluteValue < 2.0) 0.0
        else if (motor.positionError.value < 0.0) 10.0
        else -10.0
    }

    /**
     * In progress simulation of the climb
     */
    private val sim =
        Mechanism2d((Constants.MID2HIGH + Constants.HIGH2TRAVERSE).meters, Constants.TRAVERSAL_RUNG_HEIGHT.meters)
    private val extendPivot = sim.getRoot("extendable pivot", 0.0, 8.inches.value)
    private val extendable = extendPivot.append(
        MechanismLigament2d(
            "extenable",
            35.inches.value + leftWinch.position.value,
            22.5,
            1.0,
            Color8Bit(255, 0, 0)
        )
    )

    init {
        if (Game.sim) {
            Simulation.instance.include(this)
            leftWinch.setupSim(ElevatorFeedforward(0.2, 3.0, 5.0))
            SmartDashboard.putData("climb sim", sim)
            SmartDashboard.putData("winch", leftWinch)
        }
    }

    override fun simUpdate(dt: Time) {
        extendable.length = (35.inches.value + leftWinch.position.value)
        extendable.angle = if (staticsLifted) 90.0 else 22.0
    }

    override fun periodic() {
//        debugDashboard()
//        updateMotors()
    }

    private val swing = Differentiator()

    /**
     * Fun reaction wheel stuff. Optional, but cool if done
     */
    fun stabalize() {
        if (leftWinch.linearPosition < 5.inches) return
        val dTheta =
            swing.calculate(RobotContainer.gyro.pitch.radians).radiansPerSecond * -1.0  // note: pitch may be wrong
        val dampeningConstant = RobotContainer.op.climbStabilization
        Drivetrain.drive(
            DifferentialDriveWheelSpeeds(
                dTheta.value * dampeningConstant,
                dTheta.value * dampeningConstant
            )
        )
        Shooter.flywheel.torque = dTheta.value * dampeningConstant
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "Left Winch" to leftWinch,
//            "Left Arm" to leftExtendable,
            "Right Winch" to rightWinch,
//            "Right Arm" to rightExtendable,
            "Statics Raised" to staticsLifted
        )
    }
}
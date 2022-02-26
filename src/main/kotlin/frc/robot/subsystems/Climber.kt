package frc.robot.subsystems

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.Time
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.motorcontrol.KSimulatedESC
import frc.kyberlib.pneumatics.KSolenoid
import frc.kyberlib.simulation.Simulatable
import frc.kyberlib.simulation.Simulation
import frc.robot.Constants
import kotlin.math.absoluteValue

/**
 * Modes for what the climber is doing. Useful for LEDs and other dependencies
 */
enum class CLIMBER_STATUS {
    IDLE, ACTIVE, RISING, FALLING
}

/**
 * Mechanism representing the actuators for climbing
 */
object Climber : SubsystemBase(), Debug, Simulatable {
    init {
        println("Climber")
    }
    var status = CLIMBER_STATUS.IDLE

    // pneumatics that lift the climb arms
    private val leftArmLift = KSolenoid(0, fake = true)
    private val rightArmLift = KSolenoid(1, fake = true)

    private val armFF = ArmFeedforward(3.0, 2.0, 5.0, 8.0)
    val leftExtendable = KSimulatedESC(40).apply {
        identifier = "leftArm"
        brakeMode = true
//        kP = 0.1
//        kD = 5.0
//        kI = 0.2
//        addFeedforward(armFF)
        customControl = { bangBang(it) }
        minPosition = 0.degrees
        maxPosition = 90.degrees
        resetPosition(22.5.degrees)
        if(Game.sim) setupSim(armFF)
        voltage = 0.0
    }
    val rightExtendable = KSimulatedESC(41).apply {
        identifier = "rightArm"
        brakeMode = true
        kP = 5.0
        kD = 1.0
        addFeedforward(armFF)
        minPosition = 0.degrees
        maxPosition = 90.degrees
        resetPosition(22.5.degrees)
    }

    // winches that pull the robot up
    private val winchFF = SimpleMotorFeedforward(1.0, 10.0, 5.0)
    val leftWinch = KSimulatedESC(42).apply {
        radius = Constants.WINCH_RADIUS
        brakeMode = true
        gearRatio = Constants.WINCH_GEAR_RATIO
        customControl = { bangBang(it) }
        minLinearPosition = 0.inches
        maxLinearPosition = 30.inches
        motorType = DCMotor.getNeo550(1)
        if(Game.sim) setupSim(winchFF)
        }
    val rightWinch = KSimulatedESC(43).apply {
        radius = Constants.WINCH_RADIUS
        brakeMode = true
        gearRatio = Constants.WINCH_GEAR_RATIO
        customControl = { bangBang(it) }
        minLinearPosition = 0.inches
        maxLinearPosition = 30.inches
        motorType = DCMotor.getNeo550(1)
    }

    // public variable to get/set whether the arms are lifted
    var staticsLifted
        get() = leftArmLift.extended
        set(value) {
            leftArmLift.extended = value
            rightArmLift.extended = value
        }
    var extendableAngle
        get() = leftExtendable.position
        set(value) {
            leftExtendable.position = value
            rightExtendable.position = value
        }
    var extension
        get() = leftWinch.linearPosition
        set(value) {
            leftWinch.linearPosition = value
            rightWinch.linearPosition = value
        }

    fun updateMotors() {
        leftWinch.updateVoltage()
        leftExtendable.updateVoltage()
        rightWinch.updateVoltage()
        rightExtendable.updateVoltage()
    }

    /**
     * Bang bang control used winch position control.
     * If too low, go up hard.
     * If too high, go down hard.
     * @return voltage represented as double
     */
    fun bangBang(motor: KMotorController): Double {
        return if(motor.positionError.degrees.absoluteValue < 2.0) 0.0
        else if (motor.positionError.value < 0.0) 10.0
        else -10.0
    }

    /**
     * In progress simulation of the climb
     */
    private val sim = Mechanism2d((Constants.MID2HIGH + Constants.HIGH2TRAVERSE).meters, Constants.TRAVERSAL_RUNG_HEIGHT.meters)
    private val extendPivot = sim.getRoot("extendable pivot", 0.0, 8.inches.value)
    private val staticPivot = sim.getRoot("static pivot", 0.0, 15.5.inches.value)
    private val extendable = extendPivot.append(MechanismLigament2d("extenable", 35.inches.value + leftWinch.position.value, 22.5, 1.0, Color8Bit(255, 0, 0)))
    private val static = staticPivot.append(MechanismLigament2d("static", 31.inches.value, 0.0, 1.0, Color8Bit(255, 255, 0)))

    init {
        if (Game.sim) {
            SmartDashboard.putData("climb sim", sim)
            Simulation.instance.include(this)
        }
    }
    override fun simUpdate(dt: Time) {
        extendable.length = (35.inches.value + leftWinch.position.value)
        extendable.angle = leftExtendable.position.degrees
        static.angle = if(staticsLifted) 90.0 else 0.0
    }

    override fun periodic() {
        debugDashboard()
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "Left Winch" to leftWinch,
            "Left Arm" to leftExtendable,
            "Right Winch" to rightWinch,
            "Right Arm" to rightExtendable,
            "Statics Raised" to staticsLifted
        )
    }
}
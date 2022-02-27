package frc.robot.subsystems

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.Time
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.motorcontrol.KSimulatedESC
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.simulation.Simulatable
import kotlin.math.absoluteValue

/**
 * Mechanism representing the actuators for climbing
 */
object Climber : SubsystemBase(), Debug, Simulatable {
    /* no-op */
    init {
        println("Climber")
    }
    var status = CLIMBER_STATUS.IDLE

    /**left climb arm pneumatic (lifts the climb arms)*/
    private val leftArmLift = Solenoid(PneumaticsModuleType.CTREPCM, 0)
    /**right climb arm pneumatic (lifts the climb arms)*/
    private val rightArmLift = Solenoid(PneumaticsModuleType.CTREPCM, 1)

    /**Arm feed foreward.*/
    private val armFF = ArmFeedforward(3.0, 2.0, 5.0, 8.0)
    val leftExtendable = KSimulatedESC(0).apply {
        identifier = "leftArm"
        brakeMode = true
        kP = 5.0
        kD = 1.0
        addFeedforward(armFF)
        minPosition = 0.degrees
        maxPosition = 90.degrees
        resetPosition(22.5.degrees)
    }
    val rightExtendable = KSimulatedESC(0).apply {
        identifier = "rightArm"
        brakeMode = true
        kP = 5.0
        kD = 1.0
        addFeedforward(armFF)
        minPosition = 0.degrees
        maxPosition = 90.degrees
        resetPosition(22.5.degrees)
    }

    /** (right) winches that pull the robot up */
    private val winchFF = SimpleMotorFeedforward(1.0, 10.0, 5.0)
    /** (left) winches that pull the robot up */
    val leftWinch = KSimulatedESC(0).apply {
        radius = Constants.WINCH_RADIUS
        brakeMode = true
        gearRatio = Constants.WINCH_GEAR_RATIO
        customControl = { bangBang(it) }
        minLinearPosition = 0.inches
        maxLinearPosition = 30.inches
        }
    /** (right) winches that pull the robot up */
    val rightWinch = KSimulatedESC(0).apply {
        radius = Constants.WINCH_RADIUS
        brakeMode = true
        gearRatio = Constants.WINCH_GEAR_RATIO
        customControl = { bangBang(it) }
        minLinearPosition = 0.inches
        maxLinearPosition = 30.inches
    }

    /** public variable to get/set whether the arms are lifted */
    var staticsLifted
        get() = leftArmLift.get()
        set(value) {
            leftArmLift.set(value)
            rightArmLift.set(value)
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
        rightExtendable
    }

    /**
     * Bang bang control used winch position control.
     * If too low, go up hard.
     * If too high, go down hard.
     * @return voltage represented as double
     */
    fun bangBang(motor: KMotorController): Double {
        if(motor.linearPositionError.inches.absoluteValue < 2.0) return 0.0
        else if (motor.linearPositionError < 0.inches) return 10.0
        else return -0.0
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
        if (Game.sim)
            SmartDashboard.putData("climb sim", sim)
    }
    
    override fun simUpdate(dt: Time) {
        extendable.length = (35.inches.value + leftWinch.position.value)
        extendable.angle = leftExtendable.position.degrees
        static.angle = if(staticsLifted) 90.0 else 0.0

        leftExtendable.simUpdate(armFF, dt)
        rightExtendable.simUpdate(armFF, dt)
        leftExtendable.simPosition = leftExtendable.position.degrees.coerceIn(leftExtendable.minPosition!!.degrees, leftExtendable.maxPosition!!.degrees).degrees
        rightExtendable.simPosition = rightExtendable.position.degrees.coerceIn(rightExtendable.minPosition!!.degrees, rightExtendable.maxPosition!!.degrees).degrees

        leftWinch.simUpdate(winchFF, dt)
        rightWinch.simUpdate(winchFF, dt)
        leftWinch.simLinearPosition = leftWinch.position.value.coerceIn(leftWinch.minLinearPosition!!.value, leftWinch.maxLinearPosition!!.value).meters
        rightWinch.simLinearPosition = rightWinch.position.value.coerceIn(rightWinch.minLinearPosition!!.value, rightWinch.maxLinearPosition!!.value).meters

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
package frc.robot.subsystems

import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.pneumatics.KSolenoid
import frc.kyberlib.simulation.Simulatable
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
    var status = CLIMBER_STATUS.IDLE

    // pneumatics that lift the climb arms
    private val leftArmLift = KSolenoid(PneumaticsModuleType.REVPH, 0)
    private val rightArmLift = KSolenoid(PneumaticsModuleType.REVPH, 0)

    val leftExtendable = KSparkMax(0)
    private val rightExtendable = KSparkMax(0).apply { follow(leftExtendable) }

    // winches that pull the robot up
    val leftWinch = KSparkMax(0).apply {
        radius = Constants.WINCH_RADIUS
        brakeMode = true
        gearRatio = Constants.WINCH_GEAR_RATIO
        customControl = { bangBang(it) }
        }
    val rightWinch = KSparkMax(0).apply {
        radius = Constants.WINCH_RADIUS
        brakeMode = true
        gearRatio = Constants.WINCH_GEAR_RATIO
        customControl = { bangBang(it) }
    }

    // public variable to get/set whether the arms are lifted
    var armsLifted
        get() = leftArmLift.extended
        set(value) {
            leftArmLift.extended = value
            rightArmLift.extended = value
        }

    /**
     * Bang bang control used winch position control.
     * If too low, go up hard.
     * If too high, go down hard.
     * @return voltage represented as double
     */
    fun bangBang(motor: KMotorController): Double {
        if(motor.linearPositionError.inches.absoluteValue < 2.0)
            return 0.0
        else if (motor.linearPositionError < 0.inches) return 10.0
        else return -0.0
    }

    /**
     * In progress simulation of the climb
     */
    private val sim = Mechanism2d(10.0, 10.0)
    private val root = sim.getRoot("climb pivot", 1.0, 1.0)
    private val armSupport = root.append(MechanismLigament2d("elevator", 15.5.inches.value, 90.0));
    private val extendable = armSupport.append(MechanismLigament2d("extenable", 35.inches.value + leftWinch.position.value, 22.5))
    private val static = armSupport.append(MechanismLigament2d("static", 31.inches.value, 0.0))

    init {
        SmartDashboard.addData("climb sim", sim)
    }
    
    override fun simUpdate(dt: Double) {
        extendable.length = 35.inches.value + leftWinch.position.value
        extendable.angle = 
        // MechanismObject2d.
        // sim.setLigamentAngle("extendable", if(armsLifted) 90.0f else 10.0f)
        // sim.setLigamentLength("extenable", 30f + leftWinch.linearPosition.inches.toFloat())
    }

    override fun periodic() {
        debugDashboard()
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "Left Winch" to leftWinch,
            "Right Winch" to rightWinch,
            "Arms Raised" to armsLifted
        )
    }
}
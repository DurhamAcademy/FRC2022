package frc.robot.subsystems

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.math.filters.Differentiator
import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.pneumatics.KSolenoid
import frc.robot.Constants
import frc.robot.RobotContainer

/**
 * Mechanism representing the actuators for climbing
 */
object Climber : SubsystemBase(), Debug {
    // pneumatics that lift the climb arms
    private val climbLift = KSolenoid(6, 7, fake = false).apply {
        identifier = "left static"
    }
    private val climbLiftFollower = KSolenoid(1, 0, fake = false).apply {
        identifier = "right static"
        follow(climbLift)
    }

    private val armSafety = Debouncer(1.0, Debouncer.DebounceType.kFalling)
    var armsLifted
        get() = armSafety.calculate(climbLift.extended)
        set(value) {
            armSafety.calculate(value)
            climbLift.extended = value
        }

    /** (left) winches that pull the robot up */
    val leftWinch = KSparkMax(40).apply {
        identifier = "left winch"
        radius = Constants.WINCH_RADIUS
        brakeMode = true
        gearRatio = Constants.WINCH_GEAR_RATIO
        motorType = DCMotor.getNeo550(1)
        minLinearPosition = 0.inches
        maxLinearPosition = 24.inches
        currentLimit = 30

        kP = 15.0
        kD = 5.0
        setupSim(elevatorSystem(Constants.ROBOT_WEIGHT))
    }

    /** (right) winches that pull the robot up */
    val rightWinch = KSparkMax(41).apply {
        copyConfig(leftWinch)
        identifier = "right winch"
    }

    var extension  // public variable to control position off both the arms
        inline get() = leftWinch.linearPosition
        inline set(value) {
            leftWinch.linearPosition = value
            rightWinch.linearPosition = value
        }

    /**
     * In progress simulation of the climb
     */
    private val sim = Mechanism2d((Constants.MID2HIGH + Constants.HIGH2TRAVERSE).meters, Constants.TRAVERSAL_RUNG_HEIGHT.meters)
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
            SmartDashboard.putData("climb sim", sim)
        }
    }

    override fun simulationPeriodic() {
        extendable.length = (35.inches.value + leftWinch.position.value)
        extendable.angle = if (climbLift.extended) 90.0 else 22.0
    }


    private val swing = Differentiator()

    /**
     * Fun reaction wheel stuff. Optional, but cool if done
     */
    fun stabalize() {
        if (leftWinch.linearPosition < 5.inches) return
        val dTheta = -swing.calculate(RobotContainer.gyro.pitch)  // note: pitch may be wrong
        val dampeningConstant = RobotContainer.op.climbStabilization
        val targetTorque = dTheta.value * dampeningConstant
        Shooter.flywheel.torque = dTheta.value * dampeningConstant
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "Left Winch" to leftWinch,
            "Right Winch" to rightWinch,
            "Arms Raised" to climbLift.extended
        )
    }
}
package frc.robot.subsystems

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.Game
import frc.kyberlib.math.filters.Differentiator
import frc.kyberlib.math.randomizer
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.math.units.towards
import frc.kyberlib.math.units.transform
import frc.kyberlib.math.units.zeroPose
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.simulation.field.KField2d
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.turret.SeekTurret
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import org.photonvision.targeting.TargetCorner
import kotlin.math.atan

/**
 * Controls the turret
 */
object Turret : SubsystemBase() {

    //    override val priority: DebugFilter = DebugFilter.Max
    var isZeroed = false

    // characterization of the turret
    private val feedforward = SimpleMotorFeedforward(
        0.080522, 1.1453, 0.13704
    ) // 0.22832

    // actual turret motors
    private val controller = ProfiledPIDController(40.0, 3.0, 0.0, TrapezoidProfile.Constraints(1.0, 1.0)).apply {
        setIntegratorRange(-2.0, 2.0)
    }  // these constraints are not tested on real
    val turret = KSparkMax(11).apply {
        identifier = "turret"
        gearRatio = Constants.TURRET_GEAR_RATIO
        brakeMode = true
        currentLimit = 15

        customControl = { _: KMotorController ->
            position = clampSafePosition(positionSetpoint)
            val offsetCorrection = controller.calculate(position.rotations, positionSetpoint.rotations)
            val ff = feedforward.calculate(controller.setpoint.velocity.rotationsPerSecond.radiansPerSecond)
            if (isZeroed) (ff + offsetCorrection) else 0.0//.coerceIn(-4.0, 4.0)
        }
        setupSim(feedforward)
    }

    // angle of the turret from top view
    var fieldRelativeAngle: Angle
        get() = (turret.position + RobotContainer.navigation.heading).normalized
        set(value) { turret.position = value - RobotContainer.navigation.heading }

    init {
        defaultCommand = SeekTurret
    }

    /**
     * Makes an angle safe for the electronics to not get tangled
     */
    private fun clampSafePosition(angle: Angle): Angle {
        return (angle + 30.degrees).normalized - 30.degrees
    }

    /**
     * Allows the turret to know its position relative to everything else
     */
    fun zeroTurret() {  // zeros the robot position to looking straight aheead
        turret.resetPosition()
        isZeroed = true
    }

    val ready: Boolean  // is the Turret prepared to shoot
        get() = Limelight.targetVisible && turret.positionError.absoluteValue < Constants.TURRET_TOLERANCE

    override fun simulationPeriodic() {
        KField2d.getObject("turret").pose = Pose2d(RobotContainer.navigation.position, fieldRelativeAngle.w)
    }
}
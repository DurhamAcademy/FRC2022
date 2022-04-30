package frc.robot.subsystems

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.simulation.field.KField2d
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.turret.SeekTurret


/**
 * Status of what the turret is doing
 */
enum class TurretStatus {
    ADJUSTING, FROZEN, LOST
}

/**
 * Controls the turret
 */
object Turret : SubsystemBase(), Debug {
    var status = TurretStatus.LOST

    // characterization of the turret
    private val feedforward = SimpleMotorFeedforward(  // fixme
        0.080522, 1.1453, 0.13704
    ) // 0.22832

    // actual turret motors
    private val controller = ProfiledPIDController(40.0, 3.0, 0.0, TrapezoidProfile.Constraints(1.0, 1.0)).apply {
        setIntegratorRange(-2.0, 2.0)
    }  // these constraints are not tested on real
    private val turret = KSparkMax(11).apply {
        identifier = "turret"
        gearRatio = Constants.TURRET_GEAR_RATIO
        motorType = DCMotor.getNeo550(1)
        brakeMode = true
        currentLimit = 15

        val classic = { _: KMotorController ->
            val polarSpeeds = Drivetrain.polarSpeeds
            val rot = polarSpeeds.dTheta * 0.1.seconds
            val mov = polarSpeeds.dOrientation * 0.0.seconds
            angle = clampSafePosition(angleSetpoint + rot + mov)

            val offsetCorrection = controller.calculate(angle.rotations, angleSetpoint.rotations)
            val ff = feedforward.calculate(controller.setpoint.velocity.rotationsPerSecond.radiansPerSecond)
            if (isZeroed) (ff + offsetCorrection) else 0.0//.coerceIn(-4.0, 4.0)
        }

        customControl = classic
        setupSim(feedforward)
    }

    // angle of the turret from top view
    var fieldRelativeAngle: Angle
        get() = (turret.angle + RobotContainer.navigation.heading).normalized
        set(value) {
            turret.angle = value - RobotContainer.navigation.heading
        }
    var position: Angle
        get() = turret.angle
        set(value) { turret.angle = value }

    var percent
        get() = turret.percent
        set(value) { turret.percent = value }

    /**
     * Allows the turret to know its angle relative to everything else
     */
    fun zeroTurret() {  // zeros the robot angle to looking straight aheead
        turret.resetPosition()
        isZeroed = true
    }

    fun update() = turret.updateVoltage()
    fun stop() = turret.stop()
    init { defaultCommand = SeekTurret
    }

    /**
     * Makes an angle safe for the electronics to not get tangled
     */
    private fun clampSafePosition(angle: Angle): Angle {
        return (angle + 30.degrees).normalized - 30.degrees
    }

    var isZeroed = false
        private set
    val ready: Boolean  // is the Turret prepared to shoot
        get() = (Limelight.targetVisible && turret.angleError.absoluteValue < Constants.TURRET_TOLERANCE) || status == TurretStatus.FROZEN
    // smooths out how fast we switch between lost and found
    private val lostDebouncer = Debouncer(0.2, Debouncer.DebounceType.kBoth)  // *** this was changed from .5
    var lost = false
    // update limelight data
    override fun periodic() {
        SmartDashboard.putString("turret cmd", this.currentCommand?.name ?: "none")
        lost = lostDebouncer.calculate(!Limelight.targetVisible)
    }

    override fun simulationPeriodic() {
        KField2d.getObject("turret").pose = Pose2d(RobotContainer.navigation.position, fieldRelativeAngle.w)
    }
}
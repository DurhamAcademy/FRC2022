package frc.robot.subsystems

import com.revrobotics.ColorMatch
import com.revrobotics.ColorSensorV3
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Debug
import frc.kyberlib.command.DebugFilter
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.extensions.milliseconds
import frc.kyberlib.motorcontrol.rev.KSparkMax
import frc.kyberlib.simulation.field.KField2d
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.conveyor.Idle
import frc.robot.commands.intake.Intake


/**
 * State of the hopper. Used for LEDs and other dependencies
 */
enum class ConveyorStatus {
    EMPTY, SINGLE_GOOD, FULL_GOOD, BAD, FEEDING, IDLE, OFF
}

/**
 * Controls all aspects of the hopper.
 * Waiting for design to be finalized before code is added
 */
object Conveyor : SubsystemBase(), Debug {
    override val priority: DebugFilter = DebugFilter.Low

    // control conveyor
    val conveyor = KSparkMax(21).apply {
        identifier = "conveyor"
        reversed = true
        currentLimit = 10
        gearRatio = 1 / 5.0
    }

    var status = ConveyorStatus.IDLE

    // control motor that feeds into shooter
    val feeder = KSparkMax(30).apply {
        identifier = "feeder"
        gearRatio = 1 / 5.0
        currentLimit = 20
    }

    // feed into shooter
    fun feed() {
        status = ConveyorStatus.FEEDING
        feeder.percent = 0.6
        conveyor.percent = 0.6
    }

    // prepare to feed into shooter
    fun prepare() {
        conveyor.percent = -.1
        feeder.percent = -0.5
    }

    // chill
    fun idle() {
        status = ConveyorStatus.IDLE
        feeder.percent = -0.05
        conveyor.percent = -0.0
    }

    fun stop() {
        status = ConveyorStatus.OFF
        conveyor.stop()
        feeder.stop()
    }

    init {
        defaultCommand = Idle
    }

    // random hopper management we aren't gonna get to use :(
    private val colorSensorV3 = ColorSensorV3(I2C.Port.kOnboard)
    private val allianceColor = if(Game.alliance == DriverStation.Alliance.Red) Color.kRed else Color.kBlue
    private val enemyColor = if(Game.alliance == DriverStation.Alliance.Red) Color.kBlue else Color.kRed
    private val matcher = ColorMatch().apply {
        addColorMatch(allianceColor)
        addColorMatch(enemyColor)
    }
    fun checkForBalls() {
        val detection = colorSensorV3.color
        val result = matcher.matchColor(detection)
        result.let {
            status =    if(result.color == allianceColor) ConveyorStatus.SINGLE_GOOD
                        else ConveyorStatus.BAD
        }
    }

    override fun periodic() {
//        debugDashboard()
        // use hypothetical limelight to track balls and do shit
        if(RobotContainer.op.intakeCam) {
            val result = RobotContainer.ballMonitor.latestResult
            if (result != null) {
                if(result.hasTargets()) {
                    result.targets.forEach { ball ->
                        KField2d.addGoal(
                            RobotContainer.navigation.position.plus(ball.cameraToTarget.translation),
                            Game.time - result.latencyMillis.milliseconds,
                            "Ball",
                            Intake
                        )
                    }
                } else {
                    // indicate nothing is seen
                }
            }
        }
        // monitor hopper
        if(RobotContainer.op.autoShot) checkForBalls()
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "indexer" to conveyor,
            "feeder" to feeder,
            "status" to status.name
        )
    }
}
package kyberlib.mechanisms.drivetrain.swerve

import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import kyberlib.motorcontrol.GearRatio
import kyberlib.motorcontrol.KMotorController
import kyberlib.math.units.extensions.Length
import kyberlib.math.units.extensions.LinearVelocity
import kyberlib.math.units.extensions.metersPerSecond
import kotlin.math.PI

/**
 * Untested Class to control differential swerve module.
 * @param location offset from center of rotation in the Chassis
 * @param wheelRadius the radius of the module's wheel
 * @param topMotor the first motor of the dif swerve
 * @param bottomMotor the second motor of the dif swerve
 */
class DifferentialSwerveModule(location: Translation2d, wheelRadius: Length,
                               private val topMotor: KMotorController, private val bottomMotor: KMotorController
                                        ) : SwerveModule(location) {

    private val rotationPID = PIDController(0.07, 0.00, 0.01)
    private val feedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    /**
     * Custom control for each motor. Finds what voltage the motors should be.
     */
    private fun differentialControl(it: KMotorController): Double {
        val goal = stateSetpoint
        val ff = feedforward.calculate(it.linearVelocity.metersPerSecond, it.linearAcceleration.metersPerSecond)
        val velCorrection = it.PID.calculate(goal.speedMetersPerSecond)
        val rotationError = rotation - goal.angle
        val rotCorrection = rotationPID.calculate(rotationError.radians, goal.angle.radians)
        return ff + velCorrection + rotCorrection
    }

    init {
        topMotor.customControl = { it: KMotorController -> differentialControl(it) }
        bottomMotor.customControl = { it: KMotorController -> differentialControl(it) }
    }

    override var rotation: Rotation2d
        get() {
            val top = topMotor.position.normalized
            val bottom = bottomMotor.position.normalized
            return top.minus(bottom)
        }
        set(value) {
            stateSetpoint.angle = value
        }

    override var speed: LinearVelocity
        get() {
            val topSpeed = topMotor.linearVelocity  // gearing might make this weird
            val bottomSpeed = bottomMotor.linearVelocity
            return topSpeed + bottomSpeed
        }
        set(value) {
            stateSetpoint.speedMetersPerSecond = value.metersPerSecond
        }
}
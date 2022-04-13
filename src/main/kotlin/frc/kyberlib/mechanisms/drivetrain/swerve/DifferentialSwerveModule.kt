package frc.kyberlib.mechanisms.drivetrain.swerve

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Translation2d
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.KMotorController

/**
 * Untested Class to control differential swerve module.
 * @param location offset from center of rotation in the Chassis
 * @param wheelRadius the radius of the module's wheel
 * @param topMotor the first motor of the dif swerve
 * @param bottomMotor the second motor of the dif swerve
 */
class DifferentialSwerveModule(
    location: Translation2d, private val wheelRadius: Length,
    private val topMotor: KMotorController, private val bottomMotor: KMotorController
) : SwerveModule(location) {

    private val rotationPID = PIDController(0.07, 0.00, 0.01)

    private fun updateSpeeds() {
        val wheelSpeed = stateSetpoint.speedMetersPerSecond.metersPerSecond
        val rotationCorrection = rotationPID.calculate(stateSetpoint.angle.radians).radiansPerSecond
        val motorSpeed = wheelSpeed / wheelRadius / 2.0
        topMotor.velocity = motorSpeed + rotationCorrection
        bottomMotor.velocity = motorSpeed - rotationCorrection
    }

    override var rotation: Angle
        get() {
            val top = topMotor.position.normalized
            val bottom = bottomMotor.position.normalized
            return top.minus(bottom)
        }
        set(value) {
            stateSetpoint.angle = value.w
            updateSpeeds()
        }

    override var speed: LinearVelocity
        get() {
            val topSpeed = topMotor.linearVelocity  // gearing might make this weird
            val bottomSpeed = bottomMotor.linearVelocity
            return topSpeed + bottomSpeed
        }
        set(value) {
            stateSetpoint.speedMetersPerSecond = value.metersPerSecond
            updateSpeeds()
        }
}
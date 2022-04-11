package frc.kyberlib.mechanisms.drivetrain.dynamics

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.MecanumDriveKinematics
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds
import frc.kyberlib.math.units.extensions.Angle
import frc.kyberlib.math.units.extensions.LinearVelocity
import frc.kyberlib.math.units.extensions.degrees
import frc.kyberlib.math.units.extensions.metersPerSecond
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.motorcontrol.Voltage

class KMecanumDynamics(
    val leftFront: KMotorController, val leftBack: KMotorController, val rightFront: KMotorController, val rightBack: KMotorController,
    val locations: Array<Translation2d>, fieldRelativeOffset: Angle = 0.degrees
) : KHolonomicDriveDynamics(fieldRelativeOffset) {

    private val motors = arrayOf(leftFront, leftBack, rightFront, rightBack)
    private val kinematics = MecanumDriveKinematics(locations[0], locations[1], locations[2], locations[3])

    override fun driveRobotRelative(speeds: ChassisSpeeds) {
        drive(kinematics.toWheelSpeeds(chassisSpeeds))
    }

    fun drive(wheelSpeeds: MecanumDriveWheelSpeeds) {
        drive(
            wheelSpeeds.frontLeftMetersPerSecond.metersPerSecond,
            wheelSpeeds.rearLeftMetersPerSecond.metersPerSecond,
            wheelSpeeds.frontRightMetersPerSecond.metersPerSecond,
            wheelSpeeds.rearRightMetersPerSecond.metersPerSecond,
            )
    }

    fun drive(voltages: MecanumDriveMotorVoltages) {
        drive(voltages.frontLeftVoltage, voltages.rearLeftVoltage, voltages.frontRightVoltage, voltages.rearRightVoltage)
    }

    fun drive(leftFront: Voltage, leftBack: Voltage, rightFront: Voltage, rightBack: Voltage) {
        this.leftFront.voltage = leftFront
        this.leftBack.voltage = leftBack
        this.rightFront.voltage = rightFront
        this.rightBack.voltage = rightBack
    }

    fun drive(leftFront: LinearVelocity, leftBack: LinearVelocity, rightFront: LinearVelocity, rightBack: LinearVelocity) {
        this.leftFront.linearVelocity = leftFront
        this.leftBack.linearVelocity = leftBack
        this.rightFront.linearVelocity = rightFront
        this.rightBack.linearVelocity = rightBack
    }

    override fun stop() {
        motors.forEach { it.stop() }
    }

    override val chassisSpeeds: ChassisSpeeds
        get() = kinematics.toChassisSpeeds(wheelSpeeds)

    val wheelSpeeds: MecanumDriveWheelSpeeds
        get() = MecanumDriveWheelSpeeds(leftFront.linearVelocity.value, rightFront.linearVelocity.value, leftBack.linearVelocity.value, rightBack.linearVelocity.value)
}
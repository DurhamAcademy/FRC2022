package frc.kyberlib.mechanisms.drivetrain.dynamics

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.MAX_WHEEL_SPEEDS
import frc.kyberlib.auto.Navigator
import frc.kyberlib.command.Game
import frc.kyberlib.math.filters.Differentiator
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.mechanisms.drivetrain.swerve.SwerveModule
import kotlin.math.roundToLong

class KSwerveDynamics(vararg val swerveModules: SwerveModule, fieldRelativeOffset: Angle = 0.degrees) : KHolonomicDriveDynamics(fieldRelativeOffset) {
    val kinematics = SwerveDriveKinematics(*swerveModules.map { it.location }.toTypedArray())
    override val chassisSpeeds: ChassisSpeeds
        get() = kinematics.toChassisSpeeds(*swerveModules.map { it.state }.toTypedArray())

    init {
        Navigator.instance!!.applyKinematics(kinematics)
        instance = this
    }


    override fun drive(chassisSpeeds: ChassisSpeeds) = drive(chassisSpeeds, true)

    override fun driveRobotRelative(speeds: ChassisSpeeds) {
        drive(*kinematics.toSwerveModuleStates(speeds))
    }
    /**
     * Individually sets the states for each module
     */
    fun drive(vararg states: SwerveModuleState) {
        assert(states.size == swerveModules.size) { "The size of states(${states.size}) do no match the number of modules ${swerveModules.size}" }
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_WHEEL_SPEEDS.metersPerSecond)
        swerveModules.zip(states).forEach { it.first.state = it.second }
    }

    override fun stop() {
        swerveModules.forEach {
            it.state = it.brakeState
        }
    }
}

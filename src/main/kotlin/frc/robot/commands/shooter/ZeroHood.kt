package frc.robot.commands.shooter

import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.math.units.extensions.degrees
import frc.robot.subsystems.Shooter

object ZeroHood : CommandBase() {
    val limitSwitch = DigitalInput(0)  // todo
    override fun execute() {
        Shooter.hood.percent = -.1
    }

    override fun end(interrupted: Boolean) {
        Shooter.hood.resetPosition(15.degrees)  // todo
    }

    override fun isFinished(): Boolean = limitSwitch.get()
}
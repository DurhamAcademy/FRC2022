package frc.robot.commands.turret

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Debug
import frc.kyberlib.math.units.extensions.radiansPerSecond
import frc.robot.subsystems.Turret

class TestTurret : CommandBase() {
    init {
        addRequirements(Turret)
    }
    override fun initialize() {
        Turret.turret.percent = 1.0
        Debug.log("TT", "start")
    }

    override fun end(interrupted: Boolean) {
        Debug.log("TT", "end")
        Turret.turret.stop()
    }
}
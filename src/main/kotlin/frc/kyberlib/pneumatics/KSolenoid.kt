package frc.kyberlib.pneumatics

import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import frc.kyberlib.command.Game

class KSolenoid(module: PneumaticsModuleType, channel: Int) {
    private val native = if (Game.real) Solenoid(module, channel) else null

    var extended: Boolean = false
        get() = if (Game.real) native!!.get() else field
        set(value) {if(Game.real) native!!.set(value) else field = value}
}
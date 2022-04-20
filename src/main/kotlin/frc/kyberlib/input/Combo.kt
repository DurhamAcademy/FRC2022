package frc.kyberlib.input

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj2.command.button.Trigger

class Combo(private vararg val triggers: Trigger) : Trigger() {
    private var currentIndex = 0
    private val bounce = Debouncer(0.2, Debouncer.DebounceType.kFalling)

    override fun get(): Boolean {
        if (currentIndex == triggers.size) return true
        if(triggers[currentIndex].get()) currentIndex++
        else if(currentIndex == 0) return false
        else if(!bounce.calculate(triggers[currentIndex-1].get())) currentIndex = 0
        return false
    }

}
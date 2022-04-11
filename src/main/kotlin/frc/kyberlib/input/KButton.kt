package frc.kyberlib.input

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.button.Button
import edu.wpi.first.wpilibj2.command.button.Trigger
import java.util.function.BooleanSupplier

open class KButton(index: Int, condition: BooleanSupplier) : Button({
    if (KController.sim) simValues[index]
    else condition.asBoolean
}) {
    companion object {
        val all = mutableListOf<KButton>()
        val simValues = mutableListOf<Boolean>()
    }
    private var index = simValues.size
    init {
        all.add(this)
        simValues.add(false)
    }

    var simValue = false
        set(value) {
            field = value
            simValues[index] = value
        }
    constructor(condition: BooleanSupplier) : this(simValues.size, condition)
    constructor(joystick: Joystick, port: Int) : this(
        { joystick.getRawButton(port) }
    )
}

val Trigger.k
    get() = KButton { get() }
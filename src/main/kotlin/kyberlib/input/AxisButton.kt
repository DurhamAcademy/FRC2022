package kyberlib.input

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.button.Trigger

/**
 * Converts axis into a button. Activation is when the raw axis value is above a certain point
 * @param joystick the joystick that the axis is attached to
 * @param axis the index of the axis
 * @param condition booleanSupplier that takes in the raw value of the axis and checks if the 'button' should be activated
 */
class AxisButton(joystick: Joystick, axis: Int, condition: (value: Double) -> Boolean = {it > 0.5}) : Trigger({ condition(joystick.getRawAxis(axis)) })

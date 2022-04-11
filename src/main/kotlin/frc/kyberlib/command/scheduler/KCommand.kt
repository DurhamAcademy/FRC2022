package frc.kyberlib.command.scheduler

import edu.wpi.first.wpilibj2.command.Command

abstract class KCommand : Command {
    val requiredSubsystems = mutableSetOf<KSubsystem>()
    fun addRequirements(vararg requirements: KSubsystem) {
        requiredSubsystems.addAll(requirements)
    }
    override fun getRequirements(): MutableSet<KSubsystem> = requiredSubsystems
}
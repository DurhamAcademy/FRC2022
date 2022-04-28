package frc.kyberlib

import edu.wpi.first.wpilibj.Filesystem
import frc.kyberlib.math.units.Translation2d
import frc.kyberlib.math.units.extensions.*
import java.nio.file.Path

// note: this while is still a work in progress
val PATHWEAVER_PATH: Path = Filesystem.getDeployDirectory().toPath().resolve("PathWeaver")   // for TrajectoryManager

val FIELD_SIZE = Translation2d(648.inches, 324.inches)   // for KField  // fixme
val TRACK_WIDTH = 1.meters  // for diff drive stuff
val MAX_WHEEL_SPEEDS = 5.metersPerSecond

val autoTime = 15.seconds
val matchTime = 0.seconds // fixme
val endgameTime = 30.seconds
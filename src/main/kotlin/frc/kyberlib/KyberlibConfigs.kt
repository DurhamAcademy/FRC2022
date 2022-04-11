package frc.kyberlib

import frc.kyberlib.math.units.Translation2d
import frc.kyberlib.math.units.extensions.Time
import frc.kyberlib.math.units.extensions.inches
import frc.kyberlib.math.units.extensions.meters
import frc.kyberlib.math.units.extensions.seconds

val PATHWEAVER_PATH = "/Pathweaver"
val MUSIC = "/music"

val FIELD_SIZE = Translation2d(648.inches, 324.inches)
val TRACK_WIDTH = 0.6.meters

val autoTime = 15.seconds
val matchTime:Time = 0.seconds // fixme
val endgameTime = 30.seconds
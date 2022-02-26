package frc.kyberlib

import edu.wpi.first.wpilibj.Filesystem

object KyberlibConfig {
    var TRAJECTORY_PATH: String = Filesystem.getDeployDirectory().path
}

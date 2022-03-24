package frc.kyberlib.motorcontrol

typealias CANKey = String
typealias CANId = Int
typealias PWMId = Int

// storage what is using each CANId
val PWMRegristry = mutableMapOf<String, PWMId>()
val CANRegistry = mutableMapOf<CANKey, CANId>()
package kyberlib.motorcontrol

typealias CANKey = String
typealias CANId = Int

// storage what is using each CANId
val CANRegistry = mutableMapOf<CANKey, CANId>()
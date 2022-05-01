package frc.kyberlib.mechanisms.drivetrain.swerve

import com.ctre.phoenix.motorcontrol.*
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.LinearSystemSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.kyberlib.math.units.extensions.*
import frc.kyberlib.motorcontrol.GearRatio
import frc.kyberlib.motorcontrol.KMotorController
import frc.kyberlib.simulation.Simulatable
import kotlin.math.PI


typealias DifferentialModel = LinearSystem<N3, N2, N3>
/**
 * Wrapper class for a differential swerve module using LQR as the controller for azimuth angle
 * and wheel angularVelocity of the module.
 *
 * link: https://www.chiefdelphi.com/t/paper-a-state-space-model-for-a-differential-swerve/396496
 * @author Dennis Slobodzian - original (10/11/20) & TateStaples - Kotlin/Kyberlib (4/29/22)
 */
class DifferentialSwerveModule(location: Translation2d,  // todo: allow for non-statespace control option
                               val topMotor: KMotorController, val bottomMotor: KMotorController,
                               val moduleGearing: GearRatio, val moduleToWheel: GearRatio, val wheelRadius: Length,
                               model: DifferentialModel): SwerveModule(location), Simulatable {
    private val FEED_FORWARD = 0.0//12.0 / (topMotor.motorType!!.freeSpeedRadPerSec / (topMotor.gearRatio * moduleGearing * moduleToWheel))

    // Creates a Kalman Filter as our Observer for our module. Works since system is linear.
    private val observer: KalmanFilter<N3, N2, N3> = KalmanFilter(
        Nat.N3(), Nat.N3(), model,
        Matrix.mat(Nat.N3(), Nat.N1()).fill(1.0, 25.0, 5.0), // model standard errors
        Matrix.mat(Nat.N3(), Nat.N1()).fill( // measurement standard errors
            .01,// / (topMotor.gearRatio * moduleGearing),
            .1,// / (topMotor.gearRatio * moduleGearing),
            .1,// / (topMotor.gearRatio * moduleGearing * moduleToWheel),
        ),
        0.02
    )
    // Creates an LQR controller for our Swerve Module.
    private val optimizer = LinearQuadraticRegulator(
        model,
        VecBuilder.fill(0.08, 1.1, 5.0), // Q Vector/Matrix Maximum error tolerance
        VecBuilder.fill(12.0,12.0),  // R Vector/Matrix Maximum control effort.
        0.02
    )

    // Creates a LinearSystemLoop that contains the Model, Controller, Observer, Max Volts, Update Rate.
    private val loop = LinearSystemLoop(model, optimizer, observer, 12.0, 0.02)

    // periodic loop runs at 5ms.  // todo: implement notifier
    fun update() {
        optimizedState = stateSetpoint
    }
    private fun error(): Matrix<N3?, N1?>? {
        val angleError: Double = reference[0, 0] - rotation.value
        val positionError: Double = MathUtil.angleModulus(angleError)
        SmartDashboard.putNumber("err", positionError.radians.degrees)
        val error = loop.nextR.minus(loop.xHat)
        return VecBuilder.fill(positionError, error[1, 0], error[2, 0])
    }

    // use custom predict() function for as absolute encoder azimuth angle and the angular angularVelocity of the module need to be continuous.
    private fun predict() {
        // creates our input of voltage to our motors of u = K(r-x) but need to wrap angle to be continuous see wrapAngle().
        val ff = VecBuilder.fill(0.0, 0.0)//FEED_FORWARD * reference.get(2, 0), FEED_FORWARD * reference.get(2, 0))
        val u = loop.clampInput(loop.controller.k.times(error()).plus(ff))
        loop.observer.predict(u, 0.02)
//        loop.predict(0.02)
//        val u = loop.u
        topMotor.voltage = u[0,0]
        bottomMotor.voltage = u[1,0]
    }

    override val rotation: Angle get() = if(real) (topMotor.angle + bottomMotor.angle) / moduleGearing else simRot  // todo: replace with absolute encoder if included
    private inline val wheelAngularVelocity: AngularVelocity get() = if(real) (topMotor.angularVelocity - bottomMotor.angularVelocity) / moduleGearing else simVel
    override val speed: LinearVelocity get() = wheelAngularVelocity * wheelRadius
    private val moduleVelocity: AngularVelocity get() = if(real) (topMotor.angularVelocity - bottomMotor.angularVelocity) / moduleGearing else simVel

    private var reference = VecBuilder.fill(0.0, 0.0, 0.0)
    override var optimizedState: SwerveModuleState
        get() = SwerveModuleState(speed.value, rotation.w)
        set(value) {
//            VecBuilder.fill(state.angle.radians, loop.getXHat(1), state.speedMetersPerSecond / wheelRadius.meters)
            // sets the next reference
            reference = VecBuilder.fill(value.angle.radians, moduleVelocity.value, value.speedMetersPerSecond / wheelRadius.meters)
            loop.nextR = reference
            // updates the kalman filter with new data points.
            loop.correct(VecBuilder.fill(rotation.value, moduleVelocity.value, wheelAngularVelocity.value))
            // predict step of kalman filter.
            predict()
        }

    // states: module rotation, module rotation rate, wheel angular angularVelocity
    // input: left volt, right volt
    // output: module rotation, wheel angular angularVelocity
    companion object {   // check directionality of these modules
        /**
         * Creates a StateSpace model of a differential swerve module.
         *
         * @param motor is the motor used.
         * @param Js is the Moment of Inertia of the steer component.
         * @param Jw is the Moment of Inertia of the wheel component.
         * @param Gs is the Gear Ratio of the steer.
         * @param Gw is the Gear Ratio of the wheel.
         * @return LinearSystem of state space model.
         */
        fun model(motor: DCMotor, Js: Double, Jw: Double, Gs: Double, Gw: Double): LinearSystem<N3, N2, N3> {
            val Cs = -((Gs * motor.KtNMPerAmp) / (motor.KvRadPerSecPerVolt * motor.rOhms * Js));
            val Cw = -((Gw * motor.KtNMPerAmp) / (motor.KvRadPerSecPerVolt * motor.rOhms * Jw));
            val Vs = 0.5 * ((Gs * motor.KtNMPerAmp) / (motor.rOhms * Js));
            val Vw = 0.5 * ((Gw * motor.KtNMPerAmp) / (motor.rOhms * Jw));
            val A: Matrix<N3, N3> = Matrix.mat(Nat.N3(), Nat.N3()).fill(
                0.0, 1.0, 0.0,
                0.0, Gs * Cs, 0.0,
                0.0, 0.0, Gw * Cw
            )
            val B: Matrix<N3, N2> = Matrix.mat(Nat.N3(), Nat.N2()).fill(
                0.0, 0.0,
                Vs, Vs,
                Vw, -Vw
            )
            val C: Matrix<N3, N3> = Matrix.mat(Nat.N3(), Nat.N3()).fill(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)  // identity matrix
            val D: Matrix<N3, N2> = Matrix.mat(Nat.N3(), Nat.N2()).fill(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return LinearSystem(A, B, C, D)
        }
        fun model(moduleFF: SimpleMotorFeedforward, linearFF: SimpleMotorFeedforward, Gs: Double, Gw: Double): LinearSystem<N3, N2, N3> {
            //-G * G * motor.KtNMPerAmp / (motor.KvRadPerSecPerVolt * motor.rOhms * jKgMetersSquared)) = -kV / kA
            // todo: check if this will work
            val A: Matrix<N3, N3> = Matrix.mat(Nat.N3(), Nat.N3()).fill(// deriv of state = state x this
                0.0, 1.0, 0.0,
                0.0, Gs * (-moduleFF.kv/moduleFF.ka) , 0.0,
                0.0, 0.0, Gw * (-linearFF.kv/linearFF.ka)
            )
            val B: Matrix<N3, N2> = Matrix.mat(Nat.N3(), Nat.N2()).fill(// deriv of state = input x this
                0.0, 0.0,
                Gs * 1/moduleFF.ka, Gs * 1/moduleFF.ka,
                Gw * 1/linearFF.ka, Gw * 1/linearFF.ka
            )
            val C: Matrix<N3, N3> = Matrix.mat(Nat.N3(), Nat.N3()).fill(// output = state x this
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            )
            val D: Matrix<N3, N2> = Matrix.mat(Nat.N3(), Nat.N2()).fill(// output = input x this (direct feed through)
                0.0, 0.0,
                0.0, 0.0,
                0.0, 0.0
            )
            return LinearSystem(A, B, C, D)
        }
    }

    /**
     * FF based constructor
     * @param location the angle of the module relative to the center of rotation
     * @param topMotor the motor controlling the top gear of the differential
     * @param bottomMotor the motor controlling the bottom gear of the differential
     * @param moduleGearing the gear ratio between the differential gears and the module output
     * @param moduleToWheel any additional gear ratio between the module and the spinning of the wheel
     * @param wheelRadius the radius of your wheel
     * @throws MotorUnconfigure if you have not set the motorType before use
     *
     * @param moduleFF the ff value of rotating the module
     * @param linearFF the ff values on driving the module forward
     */
    constructor(location: Translation2d, topMotor: KMotorController, bottomMotor: KMotorController, moduleGearing: GearRatio, moduleToWheel: GearRatio, wheelRadius: Length,
                moduleFF: SimpleMotorFeedforward, linearFF: SimpleMotorFeedforward
    ) : this(location, topMotor, bottomMotor, moduleGearing, moduleToWheel, wheelRadius, model(moduleFF, linearFF, moduleGearing, moduleGearing * moduleToWheel))

    /**
     * FF based constructor
     * @param location the angle of the module relative to the center of rotation
     * @param topMotor the motor controlling the top gear of the differential
     * @param bottomMotor the motor controlling the bottom gear of the differential
     * @param moduleGearing the gear ratio between the differential gears and the module output
     * @param moduleToWheel any additional gear ratio between the module and the spinning of the wheel
     * @param wheelRadius the radius of your wheel
     * @throws MotorUnconfigure if you have not set the motorType before use
     *
     * @param systemMotors DCMotor represent both of the motors. Important for physics calcs
     * @param Js moment of inertia of the module
     * @param Jw moment of inertia of the wheel // todo: check how this relates to mass (check drivetrain id)
     */
    constructor(location: Translation2d, topMotor: KMotorController, bottomMotor: KMotorController, moduleGearing: GearRatio, moduleToWheel: GearRatio, wheelRadius: Length,
                systemMotors: DCMotor, Js: Double, Jw: Double
    ) : this(location, topMotor, bottomMotor, moduleGearing, moduleToWheel, wheelRadius, model(systemMotors, Js, Jw, moduleGearing, moduleGearing * moduleToWheel))

    fun debug() {
        SmartDashboard.putNumber("$identifier/rot", rotation.value)
        SmartDashboard.putNumber("$identifier/spin", moduleVelocity.value)
        SmartDashboard.putNumber("$identifier/speed", speed.value)
    }

    private val real = topMotor.real
    private var simVel = 0.radiansPerSecond
    private var simRot = 0.degrees
    private var simSpin = 0.radiansPerSecond
    private val simulation = LinearSystemSim(model)
    override fun simUpdate(dt: Time) {
        simulation.setInput(topMotor.voltage, bottomMotor.voltage)
        simulation.update(dt.seconds)
        val o = simulation.output
        simRot = o[0, 0].radians  // angle
        simSpin = o[1, 0].radiansPerSecond
        simVel = o[2, 0].radiansPerSecond  // angularVelocity
    }
}

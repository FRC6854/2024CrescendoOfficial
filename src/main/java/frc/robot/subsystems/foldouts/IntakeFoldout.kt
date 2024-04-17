package frc.robot.subsystems.foldouts

import com.ctre.phoenix6.Utils
import com.revrobotics.*
import com.revrobotics.CANSparkBase.ControlType
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Tools
import frc.robot.subsystems.light.Neopixels
import mu.KotlinLogging
import kotlin.math.abs

/**
 * Subsystem for controlling the intake foldout (its motor and states).
 * It uses a SparkMAX motor controller with a relative encoder and closed loop control.
 */
class IntakeFoldout : SubsystemBase() {
    private val logger = KotlinLogging.logger("IntakeFoldout")
    private val motor = CANSparkMax(Constants.IntakeFoldout.canID, CANSparkLowLevel.MotorType.kBrushless)
    private val controller: SparkPIDController = motor.getPIDController()

    private val encoder: RelativeEncoder = motor.encoder
    private val throughBoreEncoder = DutyCycleEncoder(Constants.IntakeFoldout.encoderPort)

    private val forwardLimitSensor = DigitalInput(Constants.IntakeFoldout.forwardLimitSensorPort)


    private var targetRotations =
        0.0 // Updates with the selectedState, easier than checking the hashmap constantly and easy publishing too
    private var selectedState: Constants.IntakeFoldout.FoldoutState = Constants.IntakeFoldout.FoldoutState.IDLE

    private var simulationLoop: PIDController? = null
    private var virtualEncoder: Double? = null

    // TODO: Add Constants for the positions
    private val mechanismCanvas = Mechanism2d(2.0, 2.0)
    private val mechanism = mechanismCanvas.getRoot("IntakeFoldout", 0.63, 0.25)
    private val pivot = mechanism.append(MechanismLigament2d("intake", 0.34, 0.0, 6.0, Color8Bit(Color.kBlue)))

    /**
     * Initializes the intake foldout, setting up the PID controller and starting the simulation loop if in simulation mode.
     */
    init {
        motor.restoreFactoryDefaults()
        //encoder.position = 0.0

        motor.enableVoltageCompensation(12.0)
        motor.setSmartCurrentLimit(30)
        motor.setIdleMode(CANSparkBase.IdleMode.kCoast) // Just for right now TODO: Change if acting icky

        controller.setP(Constants.IntakeFoldout.Control.kP)
        controller.setI(Constants.IntakeFoldout.Control.kI)
        controller.setD(Constants.IntakeFoldout.Control.kD)
        controller.setIZone(Constants.IntakeFoldout.Control.kIz)
        controller.setFF(Constants.IntakeFoldout.Control.kFF)

        controller.setSmartMotionMaxAccel(Constants.IntakeFoldout.Control.Profile.maxAcceleration, 0)
        controller.setSmartMotionMaxVelocity(Constants.IntakeFoldout.Control.Profile.maxVelocity, 0)
        controller.setSmartMotionAllowedClosedLoopError(1.0, 0)

        motor.inverted = Constants.IntakeFoldout.inverted

        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 50)
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20)
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20)
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 100)
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 1000)
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 1000)
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 1000)


        if (Utils.isSimulation()) {
            SmartDashboard.putNumber("IntakeFoldout/kP", Constants.IntakeFoldout.SimDummyControl.kP)
            SmartDashboard.putNumber("IntakeFoldout/kI", Constants.IntakeFoldout.SimDummyControl.kI)
            SmartDashboard.putNumber("IntakeFoldout/kD", Constants.IntakeFoldout.SimDummyControl.kD)
            SmartDashboard.putNumber("IntakeFoldout/kIZ", Constants.IntakeFoldout.SimDummyControl.kIz)
            SmartDashboard.putNumber("IntakeFoldout/kFF", Constants.IntakeFoldout.SimDummyControl.kFF)

            simulationLoop = PIDController(
                Constants.IntakeFoldout.SimDummyControl.kP,
                Constants.IntakeFoldout.SimDummyControl.kI,
                Constants.IntakeFoldout.SimDummyControl.kD
            )
            virtualEncoder = 0.0
        } else {
            SmartDashboard.putNumber("IntakeFoldout/kP", Constants.IntakeFoldout.Control.kP)
            SmartDashboard.putNumber("IntakeFoldout/kI", Constants.IntakeFoldout.Control.kI)
            SmartDashboard.putNumber("IntakeFoldout/kD", Constants.IntakeFoldout.Control.kD)
            SmartDashboard.putNumber("IntakeFoldout/kIZ", Constants.IntakeFoldout.Control.kIz)
            SmartDashboard.putNumber("IntakeFoldout/kFF", Constants.IntakeFoldout.Control.kFF)
        }

        if (Constants.isDebug) {
            logger.warn { "In Debug Mode, Controller Unlocked" }
        }

        motor.burnFlash()
    }

    /**
     * Updates the mechanism's rotation based on the given rotations.
     *
     * @param rotations The rotations to set the mechanism to.
     */
    private fun updateMechanism(rotations : Double) {
        pivot.setAngle(abs(rotations * Constants.IntakeFoldout.gearRatio * 360) + Constants.IntakeFoldout.offset)
    }

    /**
     * Updates the PID loops based on the values from the SmartDashboard.
     */
    private fun updateLoops() {
        val p = SmartDashboard.getNumber("IntakeFoldout/kP", Constants.IntakeFoldout.Control.kP)
        val i = SmartDashboard.getNumber("IntakeFoldout/kI", Constants.IntakeFoldout.Control.kI)
        val d = SmartDashboard.getNumber("IntakeFoldout/kD", Constants.IntakeFoldout.Control.kD)
        val iz = SmartDashboard.getNumber("IntakeFoldout/kIZ", Constants.IntakeFoldout.Control.kIz)
        val ff = SmartDashboard.getNumber("IntakeFoldout/kFF", Constants.IntakeFoldout.Control.kFF)

        if (p != Constants.IntakeFoldout.Control.kP) controller.setP(p)
        if (i != Constants.IntakeFoldout.Control.kI) controller.setI(i)
        if (d != Constants.IntakeFoldout.Control.kD) controller.setD(d)
        if (iz != Constants.IntakeFoldout.Control.kIz) controller.setIZone(iz)
        if (ff != Constants.IntakeFoldout.Control.kFF) controller.setFF(ff)
    }

    private fun updateSimLoops() {
        val p = SmartDashboard.getNumber("IntakeFoldout/kP", Constants.IntakeFoldout.SimDummyControl.kP)
        val i = SmartDashboard.getNumber("IntakeFoldout/kI", Constants.IntakeFoldout.SimDummyControl.kI)
        val d = SmartDashboard.getNumber("IntakeFoldout/kD", Constants.IntakeFoldout.SimDummyControl.kD)

        if (p != Constants.IntakeFoldout.SimDummyControl.kP) simulationLoop?.p = p
        if (i != Constants.IntakeFoldout.SimDummyControl.kI) simulationLoop?.i = i
        if (d != Constants.IntakeFoldout.SimDummyControl.kD) simulationLoop?.d = d
    }

    /**
     * Periodically updates the intake foldout, setting the reference voltage based on the selected state.
     */
    override fun periodic() {
        Tools.provideMotorDetails(motor, "IntakeFoldout")

        if (Constants.isDebug && !Utils.isSimulation()) {
            updateLoops() // PID Can be on the fly updated if on debug with SmartDashboard
        }

        // Update the reference angle from degrees to rotations
        targetRotations = (
            ((1 / Constants.IntakeFoldout.gearRatio) * ((Constants.IntakeFoldout.positions[selectedState] ?: 0.0) / 360)) - Constants.IntakeFoldout.offset / 360 / Constants.IntakeFoldout.gearRatio)

        // Set the motor to the reference angle
        // TODO: Set the motor to cancel when break is pressed
        val controllerSuccess = controller.setReference(
            targetRotations,
            ControlType.kSmartMotion
        ) // TODO: Make certain modes coast when loop is done
        
        SmartDashboard.putNumber("IntakeFoldout/Rev Error ID", controllerSuccess.value.toDouble())
        SmartDashboard.putString("IntakeFoldout/Rev Error Message", controllerSuccess.name)

        SmartDashboard.putString("IntakeFoldout/State", selectedState.toString())
        SmartDashboard.putNumber("IntakeFoldout/Setpoint", targetRotations)

        SmartDashboard.putBoolean("IntakeFoldout/forwardLimitReached", !forwardLimitSensor.get())

        if (virtualEncoder != null) {
            SmartDashboard.putNumber("IntakeFoldout/Encoder", virtualEncoder!!)
            updateMechanism(virtualEncoder!!)
        }
        else {
            SmartDashboard.putNumber("IntakeFoldout/Encoder", encoder.position)
            SmartDashboard.putNumber("IntakeFoldout/Through Bore Encoder", dutyCycleEncoder.toDouble())
            SmartDashboard.putNumber("IntakeFoldout/Velocity", encoder.velocity)
            updateMechanism(encoder.position)
        }

        SmartDashboard.putData("IntakeFoldout/Mechanism", mechanismCanvas)
    }

    /**
     * Periodically updates the intake foldout in simulation mode, using a PID controller to calculate the virtual encoder value.
     */
    override fun simulationPeriodic() {
        // REVLib sucks and doesn't have proper simulation support, this is all really basic tho

        virtualEncoder?.let {
            val changed = simulationLoop?.calculate(it, targetRotations)
            virtualEncoder = virtualEncoder!! + changed!!
            SmartDashboard.putNumber("IntakeFoldout/Sim Loop Velocity", changed*20*60)

            if (Constants.isDebug) {
                updateSimLoops()
            }
        }
    }

    /**
     * Sets the state of the intake foldout.
     *
     * @param state The desired state of the intake foldout.
     */
    fun setState(state: Constants.IntakeFoldout.FoldoutState) {
        selectedState = state
    }

    /**
     * Checks if the intake foldout is close to the target angle.
     *
     * @param target The target angle to check against.
     * @return True if the intake foldout is close to the target angle, false otherwise.
     */
    fun isClose(target : Double) : Boolean {
        return if (!Utils.isSimulation()) abs(encoder.position - target) < Constants.IntakeFoldout.completedLoop
        else {
            abs(virtualEncoder!! - target) < Constants.IntakeFoldout.completedLoop
        }
    }

    fun isClose(target: Double, tolerance: Double) : Boolean {
        return if (!Utils.isSimulation()) abs(encoder.position - target) < tolerance
        else {
            abs(virtualEncoder!! - target) < tolerance
        }
    }

    /**
     * Checks if the intake foldout is close to the target state.
     *
     * @param state The target state to check against.
     * @return True if the intake foldout is close to the target state, false otherwise.
     */
    fun isCloseToState(state: Constants.IntakeFoldout.FoldoutState) : Boolean {
        return isClose((1 / Constants.IntakeFoldout.gearRatio) * ((Constants.IntakeFoldout.positions[state] ?: 0.0) / 360))
    }

    fun isCloseToState(state: Constants.IntakeFoldout.FoldoutState, tolerance: Double) : Boolean {
        return isClose((1 / Constants.IntakeFoldout.gearRatio) * ((Constants.IntakeFoldout.positions[state] ?: 0.0) / 360), tolerance)
    }

    fun givenAngleSetState(angle : Double) {
        if (angle > Constants.IntakeFoldout.neededPositionChanges[Constants.IntakeFoldout.FoldoutState.SHOOT_HIGH]!!) {
            setState(Constants.IntakeFoldout.FoldoutState.SHOOT_HIGH)
        } else if (angle > Constants.IntakeFoldout.neededPositionChanges[Constants.IntakeFoldout.FoldoutState.SHOOT_MID]!!) {
            setState(Constants.IntakeFoldout.FoldoutState.SHOOT_MID)
        } else {
            setState(Constants.IntakeFoldout.FoldoutState.SHOOT_LOW)
        }
    }

    fun zeroEncoder(): Command {
        val command = InstantCommand({
            encoder.position = 0.0

            if (Utils.isSimulation()) {
                virtualEncoder = 0.0
            }
        }, this).ignoringDisable(true)

        command.name = "Zero IntakeFoldout Encoder"

        return command
    }

    private val dutyCycleEncoder : Float
        get() = throughBoreEncoder.absolutePosition.toFloat()

    fun mirrorEncoder() : InstantCommand {
        return InstantCommand({})
    }
}
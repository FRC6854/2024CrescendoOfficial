package frc.robot.subsystems.rotation

import com.ctre.phoenix6.Utils
import com.revrobotics.CANSparkBase.ControlType
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Tools
import mu.KotlinLogging

/**
 * Class for controlling the intake rollers of the robot.
 */
class IntakeRollers : SubsystemBase() {
    private val logger = KotlinLogging.logger("IntakeRollers")
    private val motor = CANSparkMax(Constants.IntakeRollers.canID, CANSparkLowLevel.MotorType.kBrushless)
    private val controller = motor.pidController
    private var selectedState = Constants.IntakeRollers.Speed.NEUTRAL

    private val beamBreak = DigitalInput(Constants.IntakeRollers.beamBreakID)

    private var simulationLoop: PIDController? = null
    private var virtualEncoder: Double? = null

    /**
     * Initializes the intake rollers, setting up the PID controller and starting the simulation loop if in simulation mode.
     */
    init {
        motor.restoreFactoryDefaults()

        motor.setSmartCurrentLimit(20)
        motor.enableVoltageCompensation(12.0)

        controller.setP(Constants.IntakeRollers.Control.kP)
        controller.setI(Constants.IntakeRollers.Control.kI)
        controller.setD(Constants.IntakeRollers.Control.kD)
        controller.setIZone(Constants.IntakeRollers.Control.kIz)
        controller.setFF(Constants.IntakeRollers.Control.kFF)
        controller.setOutputRange(
            Constants.IntakeRollers.Control.kMinOutput,
            Constants.IntakeRollers.Control.kMaxOutput
        )

        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 50)
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 50)
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20)
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500)
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 1000)
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 1000)
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 1000)

        /* There's gonna be no on the fly tuning support cause it's not rlly needed */

        if (Utils.isSimulation()) {
            simulationLoop = PIDController(
                Constants.IntakeRollers.Control.kP,
                Constants.IntakeRollers.Control.kI,
                Constants.IntakeRollers.Control.kD
            )
            virtualEncoder = 0.0
        }
    }

    /**
     * Sets the state of the intake rollers.
     *
     * @param state The desired state of the intake rollers.
     */
    fun setState(state: Constants.IntakeRollers.Speed) {
        selectedState = state
    }

    /**
     * Checks if the beam break sensor is broken.
     *
     * @return True if the beam break sensor is broken, false otherwise.
     */
    fun isBroken() = !beamBreak.get()

    /**
     * Periodically updates the intake rollers, setting the reference voltage based on the selected state.
     */
    override fun periodic() {
        Tools.provideMotorDetails(motor, "IntakeRollers")

        Constants.IntakeRollers.speeds[selectedState]?.let {
            controller.setReference(
                (it / 100) * 12,
                ControlType.kVoltage
            )
        }

        if (virtualEncoder != null) SmartDashboard.putNumber("IntakeRollers/Encoder", virtualEncoder!!) else SmartDashboard.putNumber("IntakeRollers/Encoder", motor.encoder.velocity)
        SmartDashboard.putString("IntakeRollers/State", selectedState.name)
        SmartDashboard.putBoolean("IntakeRollers/Sensor", isBroken())
    }

    /**
     * Periodically updates the intake rollers in simulation mode, using a PID controller to calculate the virtual encoder value.
     */
    override fun simulationPeriodic() {
        simulationLoop?.let {
            Constants.IntakeRollers.speeds[selectedState]?.let { speed ->
                virtualEncoder = virtualEncoder!! + it.calculate(virtualEncoder!!, (speed / 100) * 12)
                SmartDashboard.putNumber("IntakeRollers/Sim Loop Velocity", (virtualEncoder!! / 12) * 5400)
            }
        }
    }
}
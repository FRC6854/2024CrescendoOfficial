package frc.robot.subsystems.climb

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants

class Climber() : SubsystemBase() {
    private val leftMotor = TalonFX(Constants.Climber.leftMotorID)
    private val rightMotor = TalonFX(Constants.Climber.rightMotorID)
    private val disabledLimits = false

    private val request = DutyCycleOut(0.0)

    enum class Motor {
        LEFT, RIGHT
    }

    init {
        leftMotor.inverted = Constants.Climber.isLeftInverted
        rightMotor.inverted = Constants.Climber.isRightInverted

        request.OverrideBrakeDurNeutral = true

        leftMotor.setNeutralMode(NeutralModeValue.Brake)
        rightMotor.setNeutralMode(NeutralModeValue.Brake)

        // Soft limits
        val climberLimit = SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(Constants.Climber.maximumPosition)
            .withReverseSoftLimitThreshold(Constants.Climber.minimumPosition)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)

        leftMotor.configurator.apply(climberLimit)
        rightMotor.configurator.apply(climberLimit)
    }

    fun setMotor(motor: Motor, percent: Double) {
        val motorInstance = when (motor) {
            Motor.LEFT -> leftMotor
            Motor.RIGHT -> rightMotor
        }

        val currentPos = motorInstance.position
        val output = if (currentPos.valueAsDouble >= Constants.Climber.maximumPosition && percent > 0 && !disabledLimits) {
            0.0 // Prevent positive output when above maximum position, wont work if limits are disabled
        } else {
            percent
        }

        motorInstance.setControl(request.withOutput(output))
    }

    private fun setEncoderPosition(motor: Motor, position: Double) {
        if (position >= Constants.Climber.maximumPosition) {
            val increasedLimit = SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(position)
                .withReverseSoftLimitThreshold(Constants.Climber.minimumPosition)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)

            when (motor) {
                Motor.LEFT -> leftMotor.configurator.apply(increasedLimit)
                Motor.RIGHT -> rightMotor.configurator.apply(increasedLimit)
            }
        }

        when (motor) {
            Motor.LEFT -> leftMotor.setPosition(position)
            Motor.RIGHT -> rightMotor.setPosition(position)
        }
    }

    private fun disableSoftLimits() {
        val disableLimits = SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false)

        leftMotor.configurator.apply(disableLimits)
        rightMotor.configurator.apply(disableLimits)
    }

    fun setClimberToMaxPositions() : Command {
        val command = InstantCommand({
            setEncoderPosition(Motor.LEFT, Constants.Climber.theoreticalMax)
            setEncoderPosition(Motor.RIGHT, Constants.Climber.theoreticalMax)
        }, this)

        command.name = "Set Climber to Max Positions"

        return command
    }

    fun setDisableSoftLimits() : Command {
        val command = InstantCommand({
            disableSoftLimits()
        }, this)

        command.name = "Disable Soft Limits"

        return command
    }
}
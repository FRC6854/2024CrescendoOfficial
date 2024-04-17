package frc.robot.commands.climb

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.climb.Climber
import java.util.function.DoubleSupplier

class DefaultClimber(
    private val climber: Climber,
    private val leftStick: DoubleSupplier,
    private val rightStick: DoubleSupplier
) : Command() {
    init {
        addRequirements(climber)
    }

    override fun execute() {
        climber.setMotor(Climber.Motor.LEFT, leftStick.asDouble)
        climber.setMotor(Climber.Motor.RIGHT, rightStick.asDouble)
    }

    override fun isFinished() = false

    override fun end(interrupted: Boolean) {
        climber.setMotor(Climber.Motor.LEFT, 0.0)
        climber.setMotor(Climber.Motor.RIGHT, 0.0)
    }
}
package frc.robot.commands.teleop

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.foldouts.IntakeFoldout
import frc.robot.subsystems.rotation.IntakeRollers

class ShootPoop(
    private val intakeFoldout : IntakeFoldout,
    private val intakeRollers : IntakeRollers,
) : Command() {
    init {
        addRequirements(intakeFoldout, intakeRollers)
    }

    override fun initialize() {
        intakeRollers.setState(Constants.IntakeRollers.Speed.NEUTRAL)
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        intakeRollers.setState(Constants.IntakeRollers.Speed.SHOOT)
    }
}
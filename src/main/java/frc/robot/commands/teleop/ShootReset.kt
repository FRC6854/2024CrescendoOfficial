package frc.robot.commands.teleop

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.foldouts.IntakeFoldout
import frc.robot.subsystems.rotation.IntakeRollers

class ShootReset(
    private val intakeFoldout : IntakeFoldout,
    private val intakeRollers : IntakeRollers,
) : Command() {
    init {
        addRequirements(intakeFoldout, intakeRollers)
        name = "ShootReset"
    }

    override fun initialize() {
        intakeFoldout.setState(Constants.IntakeFoldout.FoldoutState.IDLE)
        intakeRollers.setState(Constants.IntakeRollers.Speed.NEUTRAL)
    }

    override fun isFinished(): Boolean {
        return true
    }
}
package frc.robot.commands.teleop

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.foldouts.IntakeFoldout
import frc.robot.subsystems.rotation.IntakeRollers

class ShootToAmp(
    private val intakeFoldout: IntakeFoldout,
    private val intakeRollers: IntakeRollers,
) : Command() {
    init {
        addRequirements(intakeFoldout, intakeRollers)
        name = "ShootToAmp"
    }

    override fun initialize() {
        intakeFoldout.setState(Constants.IntakeFoldout.FoldoutState.AMPLIFY)
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        intakeRollers.setState(Constants.IntakeRollers.Speed.AMPLIFY)
    }
}
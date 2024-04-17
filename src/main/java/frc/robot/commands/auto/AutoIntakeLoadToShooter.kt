package frc.robot.commands.auto

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.foldouts.IntakeFoldout
import frc.robot.subsystems.rotation.IntakeRollers

/**
 * Command for automatically loading the intake to the shooter.
 * This command sets the intake foldout to the low shoot state, the shooter foldout to the avoid state,
 * and the intake rollers to the neutral state.
 *
 * @property intakeFoldout The IntakeFoldout subsystem.
 * @property shooterFoldout The ShooterFoldout subsystem.
 * @property intakeRollers The IntakeRollers subsystem.
 */
class AutoIntakeLoadToShooter(
    private val intakeFoldout: IntakeFoldout,
    private val intakeRollers: IntakeRollers
) : Command() {

    /**
     * Initializes the command, adding the required subsystems and setting the command name.
     */
    init {
        addRequirements(intakeFoldout, intakeRollers)
        name = "AutoIntakeLoadToShooter"
    }

    /**
     * Called when the command is initially scheduled.
     * Sets the state of the intake foldout, shooter foldout, and intake rollers.
     */
    override fun initialize() {
        intakeFoldout.setState(Constants.IntakeFoldout.FoldoutState.IDLE)
        intakeRollers.setState(Constants.IntakeRollers.Speed.NEUTRAL)
    }

    /**
     * Returns whether the command has finished.
     * The command is finished when the intake foldout is close to the low shoot state.
     *
     * @return True if the command is finished, false otherwise.
     */
    override fun isFinished(): Boolean {
        return intakeFoldout.isCloseToState(Constants.IntakeFoldout.FoldoutState.IDLE)
    }
}
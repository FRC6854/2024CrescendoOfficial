package frc.robot.commands.teleop

import com.ctre.phoenix6.Utils
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.foldouts.IntakeFoldout
import frc.robot.subsystems.rotation.IntakeRollers

/**
 * Command for intaking from the ground during teleop.
 * This command sets the intake foldout to the ground intake state, the shooter foldout to the avoid state,
 * and the intake rollers to the intake state.
 *
 * @property intakeFoldout The IntakeFoldout subsystem.
 * @property intakeRollers The IntakeRollers subsystem.
 */
class IntakeFromGround(
    private val intakeFoldout: IntakeFoldout,
    private val intakeRollers: IntakeRollers,
) : Command() {
    private var simulationTicks : Int? = null

    /**
     * Initializes the command, adding the required subsystems and setting the command name.
     */
    init {
        addRequirements(intakeFoldout, intakeRollers)
        name = "IntakeFromGround"
    }

    /**
     * Called when the command is initially scheduled.
     * Sets the state of the intake foldout, shooter foldout, and intake rollers.
     */
    override fun initialize() {
        intakeFoldout.setState(Constants.IntakeFoldout.FoldoutState.INTAKE_GROUND)
        intakeRollers.setState(Constants.IntakeRollers.Speed.INTAKE)

        if (Utils.isSimulation()) {
            simulationTicks = 0
        }
    }

    /**
     * Returns whether the command has finished.
     * The command is finished when the intake rollers are broken or the simulation time has passed.
     *
     * @return True if the command is finished, false otherwise.
     */
    override fun isFinished(): Boolean {
        if (simulationTicks != null) {
            simulationTicks = simulationTicks?.plus(1)
            return simulationTicks!! > Constants.IntakeFoldout.simulationTime
        }
        return intakeRollers.isBroken()
    }

    /**
     * Called once after isFinished returns true or the command is interrupted.
     * Sets the state of the intake foldout, shooter foldout, and intake rollers.
     *
     * @param interrupted Whether the command was interrupted.
     */
    override fun end(interrupted: Boolean) {
        intakeRollers.setState(Constants.IntakeRollers.Speed.NEUTRAL)
        intakeFoldout.setState(Constants.IntakeFoldout.FoldoutState.IDLE)
    }
}
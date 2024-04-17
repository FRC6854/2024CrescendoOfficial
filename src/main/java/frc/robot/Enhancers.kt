package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.commands.teleop.IntakeFromSource
import frc.robot.subsystems.foldouts.IntakeFoldout
import frc.robot.subsystems.rotation.IntakeRollers

/**
 * On the Fly Path Generation (aka Enhancers) for the robot
 */
object Enhancers {
    /**
     * Generate a path to the source
     * @return The path to the source
     */
    fun generateSourcePath() : Command {
        if (!Constants.supportEnhancers) return InstantCommand()

        val path = PathPlannerPath.fromPathFile(Constants.Enhancers.Pathfinding.approachSourcePath)
        return AutoBuilder.pathfindThenFollowPath(path, Constants.Enhancers.Pathfinding.constraints, 0.0).withName("Enhancer to Source")
    }

    /**
     * Generate a path to the amp
     * @return The path to the amp
     */

    fun generateAmpPath() : Command {
        if (!Constants.supportEnhancers) return InstantCommand()

        val path = PathPlannerPath.fromPathFile(Constants.Enhancers.Pathfinding.approachAmpPath)
        return AutoBuilder.pathfindThenFollowPath(path, Constants.Enhancers.Pathfinding.constraints, 0.0).withName("Enhancer to Amp")
    }

    /**
     * Generate a path to the speaker (left)
     * @return The path to the speaker (left)
     */
    fun generateSpeakerPathLeft() : Command {
        if (!Constants.supportEnhancers) return InstantCommand()

        val path = PathPlannerPath.fromPathFile(Constants.Enhancers.Pathfinding.approachSpeakerPathLeft)
        return AutoBuilder.pathfindThenFollowPath(path, Constants.Enhancers.Pathfinding.constraints, 0.0).withName("Enhancer to Speaker (Left)")
    }

    /**
     * Generate a path to the speaker (right)
     * @return The path to the speaker (right)
     */
    fun generateSpeakerPathRight() : Command {
        if (!Constants.supportEnhancers) return InstantCommand()

        val path = PathPlannerPath.fromPathFile(Constants.Enhancers.Pathfinding.approachSpeakerPathRight)
        return AutoBuilder.pathfindThenFollowPath(path, Constants.Enhancers.Pathfinding.constraints, 0.0).withName("Enhancer to Speaker (Right)")
    }

    /**
     * Generate a cycle path, this is EXPERIMENTAL and should not be used in competition
     * @return The cycle path
     */
    fun generateCyclePath(intakeFoldout: IntakeFoldout, intakeRollers: IntakeRollers) : SequentialCommandGroup {
        if (!Constants.supportEnhancers) return SequentialCommandGroup()

        if (Constants.isDebug) {
            return SequentialCommandGroup(
                generateSourcePath(),
                IntakeFromSource(intakeFoldout, intakeRollers), // Wait until beam break sensor is triggered
                WaitCommand(0.5),
                generateSpeakerPathRight()
            )
        }
        return SequentialCommandGroup(generateSpeakerPathRight())
    }
}
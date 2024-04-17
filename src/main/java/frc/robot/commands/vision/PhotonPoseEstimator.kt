package frc.robot.commands.vision

import com.ctre.phoenix6.Utils
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.CommandSwerveDrivetrain
import frc.robot.Telemetry
import frc.robot.Tools
import frc.robot.subsystems.vision.PhotonVision
import kotlin.concurrent.thread
import kotlin.math.abs

class PhotonPoseEstimator(
    private val drivetrain: CommandSwerveDrivetrain,
    private val telemetry: Telemetry,
    private val vision: PhotonVision
) : Command() {
    private val isSimulation = Utils.isSimulation()

    override fun initialize() {
        SmartDashboard.putNumberArray("Left Camera", arrayOf(0.0, 0.0, 0.0))
        SmartDashboard.putNumberArray("Right Camera", arrayOf(0.0, 0.0, 0.0))

        addRequirements(vision)
    }

    override fun execute() {
        thread {
            val (left, right) = vision.getCameraResults()

            if (left.isPresent) addPosition(left.best, "Left Camera")
            if (right.isPresent) addPosition(right.best, "Right Camera")
        }
    }

    private fun addPosition(suggestedPosition: Transform3d, name : String) {
        val dimensionalPose = Tools.transform3DtoPose2D(suggestedPosition)
        val timestamp = Timer.getFPGATimestamp()

        if (abs(Tools.distanceBetweenPoses(telemetry.position, dimensionalPose)) < 1) { // Suggested by CTRE Devs, may have issues if robot vision doesn't work the whole round and then starts back up?
            if (!isSimulation) drivetrain.addVisionMeasurement(dimensionalPose, timestamp)
            SmartDashboard.putNumberArray(name, arrayOf(dimensionalPose.translation.x, dimensionalPose.translation.y, dimensionalPose.rotation.degrees))
        }
    }

    override fun runsWhenDisabled(): Boolean {
        return true
    }

    override fun isFinished(): Boolean {
        return false
    }
}
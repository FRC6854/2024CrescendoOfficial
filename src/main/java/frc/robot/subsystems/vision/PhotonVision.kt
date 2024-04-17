package frc.robot.subsystems.vision

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation2d
import org.photonvision.simulation.VisionSystemSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.PhotonCamera
import com.ctre.phoenix6.Utils
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.Constants
import frc.robot.Telemetry
import org.photonvision.targeting.PNPResult

/**
 * Main vision system of the robot.
 * Packages simulator and real robot vision into one interface.
 */
class PhotonVision(private val telemetry : Telemetry) : SubsystemBase() {
    /**
     * The layout of the AprilTag field.
     */
    private val tagLayout: AprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile)

    /**
     * The left camera used by the PhotonVision system.
     * This is initialized with the name specified in the Constants.Vision.LeftCamera.name.
     */
    private val leftCamera = PhotonCamera(Constants.Vision.LeftCamera.name)

    /**
     * The right camera used by the PhotonVision system.
     * This is initialized with the name specified in the Constants.Vision.RightCamera.name.
     */
    private val rightCamera = PhotonCamera(Constants.Vision.RightCamera.name)

    /* Simulation properties */
    private var visionSim: VisionSystemSim? = null
    private var cameraSimProp: SimCameraProperties? = null
    private var leftCameraSim: PhotonCameraSim? = null
    private var rightCameraSim: PhotonCameraSim? = null

    init {
        if (Utils.isSimulation() && Constants.Vision.enable) {
            setupSimulation()
        }
    }

    /**
     * Retrieves the latest pose estimation results from both the left and right cameras.
     *
     * @return A pair of PNPResult objects, where the first element is the result from the left camera and the second element is the result from the right camera.
     */
    fun getCameraResults() : Pair<PNPResult, PNPResult> {
        return Pair(
            leftCamera.latestResult.multiTagResult.estimatedPose,
            rightCamera.latestResult.multiTagResult.estimatedPose
        )
    }

    /**
     * Sets up the simulation.
     */
    private fun setupSimulation() {
        visionSim = VisionSystemSim("photonvision")
        visionSim?.addAprilTags(tagLayout)

        cameraSimProp = SimCameraProperties().apply {
            setCalibration(Constants.Simulation.Camera.width, Constants.Simulation.Camera.height, Rotation2d.fromDegrees(Constants.Simulation.Camera.fov.toDouble()))
            setCalibError(Constants.Simulation.Camera.averageNoise, Constants.Simulation.Camera.deviatedNoise)
            fps = Constants.Simulation.Camera.fps.toDouble()
            avgLatencyMs = Constants.Simulation.Camera.averageLatency.toDouble()
            latencyStdDevMs = Constants.Simulation.Camera.standardDevLatency.toDouble()
        }

        leftCameraSim = PhotonCameraSim(leftCamera, cameraSimProp)
        rightCameraSim = PhotonCameraSim(rightCamera, cameraSimProp)

        visionSim?.addCamera(leftCameraSim, Constants.Vision.LeftCamera.robotToCamera)
        visionSim?.addCamera(rightCameraSim, Constants.Vision.RightCamera.robotToCamera)

        visionSim?.debugField

        leftCameraSim?.enableDrawWireframe(true)
        rightCameraSim?.enableDrawWireframe(true)
    }

    /**
     * Updates the simulation periodically.
     */
    override fun simulationPeriodic() {
        visionSim?.update(telemetry.position) // When disabled and in the simulation, Kotlin's null safety will prevent this from running
    }
}
package frc.robot

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.util.datalog.DoubleArrayLogEntry
import edu.wpi.first.util.datalog.DoubleLogEntry
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit

import mu.KotlinLogging;

/**
 * Class for telemetry data collection and publishing.
 *
 * @property MaxSpeed Maximum speed of the robot in meters per second.
 */
class Telemetry(private val MaxSpeed: Double) {
    private val logger = KotlinLogging.logger {}
    private val logEntry: DoubleArrayLogEntry
    private val odomEntry: DoubleLogEntry

    /* What to publish over networktables for telemetry */
    private val inst = NetworkTableInstance.getDefault()

    /* Robot pose for field positioning */
    private val table = inst.getTable("Pose")
    private val fieldPub = table.getDoubleArrayTopic("robotPose").publish()
    private val fieldTypePub = table.getStringTopic(".type").publish()

    /* Robot speeds for general checking */
    private val driveStats = inst.getTable("Drive")
    private val velocityX = driveStats.getDoubleTopic("Velocity X").publish()
    private val velocityY = driveStats.getDoubleTopic("Velocity Y").publish()
    private val speed = driveStats.getDoubleTopic("Speed").publish()
    private val odomPeriod = driveStats.getDoubleTopic("Odometry Period").publish()

    /* Keep a reference of the last pose to calculate the speeds */
    private var m_lastPose = Pose2d()
    private var lastTime = Utils.getCurrentTimeSeconds()

    /* Mechanisms to represent the swerve module states */
    private val m_moduleMechanisms = arrayOf(
        Mechanism2d(1.0, 1.0),
        Mechanism2d(1.0, 1.0),
        Mechanism2d(1.0, 1.0),
        Mechanism2d(1.0, 1.0)
    )

    /* A direction and length changing ligament for speed representation */
    private val m_moduleSpeeds = arrayOf(
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(MechanismLigament2d("Speed", 0.5, 0.0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(MechanismLigament2d("Speed", 0.5, 0.0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(MechanismLigament2d("Speed", 0.5, 0.0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(MechanismLigament2d("Speed", 0.5, 0.0))
    )

    /* A direction changing and length constant ligament for module direction */
    private val m_moduleDirections = arrayOf(
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite)))
    )

    /**
     * Initializes the telemetry object and starts the SignalLogger.
     */
    init {
        SignalLogger.start()
        logEntry = DoubleArrayLogEntry(DataLogManager.getLog(), "odometry")
        odomEntry = DoubleLogEntry(DataLogManager.getLog(), "odom period")
    }

    /**
     * Publishes the current state of the swerve drive to the SmartDashboard.
     *
     * @param state The current state of the swerve drive.
     */
    fun telemeterize(state: SwerveDriveState) {
        /* Telemeterize the pose */
        val pose = state.Pose
        fieldTypePub.set("Field2d")
        fieldPub.set(
            doubleArrayOf(
                pose.x,
                pose.y,
                pose.rotation.degrees
            )
        )

        /* Telemeterize the robot's general speeds */
        val currentTime = Utils.getCurrentTimeSeconds()
        val diffTime = currentTime - lastTime
        lastTime = currentTime
        val distanceDiff = pose.minus(m_lastPose).translation
        m_lastPose = pose
        val velocities = distanceDiff.div(diffTime)
        speed.set(velocities.norm)
        velocityX.set(velocities.x)
        velocityY.set(velocities.y)
        odomPeriod.set(1.0 / state.OdometryPeriod)

        /* Telemeterize the module's states */for (i in 0..3) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle)
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle)
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed))
            SmartDashboard.putData("Swerve/Module $i", m_moduleMechanisms[i])
        }
        val timestamp = (Timer.getFPGATimestamp() * 1000000).toLong()
        logEntry.append(doubleArrayOf(pose.x, pose.y, pose.rotation.degrees), timestamp)
        odomEntry.append(state.OdometryPeriod, timestamp)
    }

    /**
     * Returns the last known pose of the robot.
     */
    val position: Pose2d
        get() {
            return m_lastPose
        }
}
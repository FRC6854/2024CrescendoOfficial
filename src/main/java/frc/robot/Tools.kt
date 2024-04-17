package frc.robot

import com.revrobotics.CANSparkMax
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.atan2
import kotlin.math.sqrt

/**
 * A class that contains useful equations and tools for the robot
 */
object Tools {
    /**
     * Calculate the angle (in a cartesian plane) the robot should be facing to shoot at the target
     * @param score_pos The position of the target
     * @param robot_pose The position of the robot
     * @return The angle the robot should be facing
     */
    fun calculateFacingAngle(score_pos: Pose3d, robot_pose: Pose2d): Double {
        val dx = score_pos.translation.x - robot_pose.translation.x // delta X
        val dy = score_pos.translation.y - robot_pose.translation.y // delta Y
        return Math.toDegrees(atan2(dy, dx)) // Tangent Angle (in Rotations)
    }

    /**
     * Calculate the distance between the robot and the target
     * @param score_pos The position of the target
     * @param robot_pose The position of the robot
     * @return The distance between the robot and the target
     */
    fun calculateDistance(score_pos: Pose3d, robot_pose : Pose2d): Double {
        val dx = score_pos.translation.x - robot_pose.translation.x // delta X
        val dy = score_pos.translation.y - robot_pose.translation.y // delta Y
        return sqrt(dx * dx + dy * dy) // Distance of B^2 = A^2 + C^2 (Find distance using Pythagorean Theorem)
    }

    /**
     * Calculate the angle the robot shooter should shoot at to hit the target
     * @param distance The distance between the robot and the target
     * @param height The height of the target
     * @return The angle the robot should shoot at
     */
    fun calculateShotAngle(distance: Double, height: Double): Double {
        val angle = Math.toDegrees(atan2(height, distance)) // Tangent Angle (in Degrees)
        return angle.coerceIn(4.0, 70.0) // Coerce the angle to be within the range of 4 to 65 degrees
    }

    /**
     * Transform a 3D transform to a 2D pose
     * @param transform The 3D transform
     * @return The 2D pose
     */
    fun transform3DtoPose2D(transform : Transform3d) : Pose2d {
        return Pose2d(transform.translation.x, transform.translation.y, Rotation2d.fromRadians(transform.rotation.z))
    }

    /**
     * Calculate the distance between two poses
     * @param pose1 The first pose
     * @param pose2 The second pose
     * @return The distance between the two poses
     */
    fun distanceBetweenPoses(pose1: Pose2d, pose2: Pose2d): Double {
        val dx = pose1.translation.x - pose2.translation.x // delta X
        val dy = pose1.translation.y - pose2.translation.y // delta Y
        return sqrt(dx * dx + dy * dy) // Distance of B^2 = A^2 + C^2 (Find distance using Pythagorean Theorem)
    }

    fun invertWhen(value: Double, condition: Boolean): Double {
        return if (condition) -value else value
    }

    fun provideMotorDetails(motor: CANSparkMax, name: String) {
        SmartDashboard.putNumber("Motors/$name/Output", motor.appliedOutput)
        SmartDashboard.putNumber("Motors/$name/Current", motor.outputCurrent)
        SmartDashboard.putNumber("Motors/$name/Voltage", motor.busVoltage)
        SmartDashboard.putNumber("Motors/$name/Temp", motor.motorTemperature)
    }
}
package frc.robot

import com.pathplanner.lib.path.PathConstraints
import frc.lib.light.Color
import frc.lib.light.NeopixelSection
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d

object Constants {
    const val isDebug = true
    const val beefySim = true
    const val supportEnhancers = false

    object Robot {
        const val name = "Viper"

        const val radius = 0.4064
        const val width = 0.8128
        const val length = 0.8128

        const val maxSpeed = 6.21 // Meters per second
        const val maxAngularSpeed = Math.PI * 1.75 // Radians per second
    }

    object Indicators {
        const val enableCameraLED = false

        object Neopixels {
            const val neopixelPort = 0
            const val numLEDs = 18
            const val enabled = true

            enum class NeopixelStates {
                /* This is configurable to allow other states */
                SHOOTING, INTAKING, IDLE, DEAD, AUTO, BREAKING
            }


            private val indicators = hashMapOf(
                NeopixelStates.IDLE to Color(0, 240, 255), // Cyan
                NeopixelStates.INTAKING to Color(0, 200, 0), // Green
                NeopixelStates.SHOOTING to Color(255, 69, 0), // Orange
                NeopixelStates.DEAD to Color(0, 12, 163), // Dark Cyan
                NeopixelStates.AUTO to Color(255, 255, 0), // Yellow
                NeopixelStates.BREAKING to Color(255, 0, 0) // Red
            )

            val sections = arrayOf(
                NeopixelSection(0, 17, "Indicators", indicators),
            )
        }
    }

    object Enhancers {
        val blueScorePosition = Pose3d(0.0, 5.5372, 2.032, Rotation3d(0.0, 0.0, 0.0))
        val redScorePosition = Pose3d(16.54, 5.5372, 2.032, Rotation3d(0.0, 0.0, Math.PI))

        object PathPlanner {
            // TODO: Tune this (again and again and again)
            object RotationGains {
                const val kP = 6
                const val kI = 0.0
                const val kD = 0.0
            }

            object TranslationGains {
                const val kP = 6
                const val kI = 0.0
                const val kD = 0.0
            }
        }

        object Pathfinding {
            val constraints = PathConstraints(6.1, 10.0, Rotation2d.fromDegrees(540.0).degrees, Rotation2d.fromDegrees(720.0).degrees)

            const val approachSourcePath = "Automatic Source Approach"
            const val approachAmpPath = "Automatic Amp Approach"
            const val approachSpeakerPathLeft = "Automatic Speaker Left Approach"
            const val approachSpeakerPathRight = "Automatic Speaker Right Approach"
        }
    }

    object Simulation {
        /* Simulation also takes data directly from Photon (Constants.Vision) */
        object Camera {
            const val width = 1280
            const val height = 720
            const val fov = 70
            const val averageNoise = 0.25
            const val deviatedNoise = .08
            const val fps = 30
            const val averageLatency = 35
            const val standardDevLatency = 3
        }
    }

    object Vision {
        const val squareConstant = 0.291846 // The robot is square so this is the distance from the center of the robot to the corner
        const val robotLift = 0.2001505 // Distance from ground to cameras

        const val enable = false

        object LeftCamera {
            const val name = "leftCam"
            val robotToCameraTrl =  Translation3d(squareConstant, squareConstant, robotLift)
            val robotToCameraRot = Rotation3d(0.0, Math.toRadians(-25.0), Math.toRadians(45.0))
            val robotToCamera = Transform3d(robotToCameraTrl, robotToCameraRot)
        }
        object RightCamera {
            const val name = "rightCam"
            val robotToCameraTrl = Translation3d(squareConstant, -squareConstant, robotLift)
            val robotToCameraRot = Rotation3d(0.0, Math.toRadians(-25.0), Math.toRadians(-45.0))
            val robotToCamera = Transform3d(robotToCameraTrl, robotToCameraRot)
        }
    }

    object IntakeFoldout {
        const val canID = 22
        const val gearRatio  = (1.0/100.0)
        const val inverted = true

        const val throughBoreCountsPerRev = 8192
        const val encoderPort = 0
        const val forwardLimitSensorPort = 1

        const val offset = 12.0 // Degrees

        const val completedLoop = 5 // The tolerance for the loop to be considered done

        const val simulationTime = 300 // The amount of ticks before a virtual beam break is triggered

        object Control {
            const val kP = 0.000135
            const val kI = 0.0
            const val kD = 0.0 // TODO: Up this
            const val kIz = 0.0
            const val kFF = 0.0

            object Profile {
                const val maxVelocity = 5400.0
                const val maxAcceleration = 120000.0
            }
        }

        object SimDummyControl {
            const val kP = .06
            const val kI = 0.0
            const val kD = 0.0
            const val kIz = 0.0
            const val kFF = 0.0
        }

        enum class FoldoutState {
            IDLE,
            SHOOT_LOW,
            SHOOT_MID,
            SHOOT_HIGH,
            INTAKE_DROP,
            INTAKE_GROUND,
            AMPLIFY,
        }

        /* Positions of the Intake Foldout States (in degrees) */
         val positions = hashMapOf(
            FoldoutState.IDLE to 40.0,
            FoldoutState.SHOOT_LOW to 25.0, // Ok for disgruntled programmer looking at this code in five years, when pressing the shoot button it will position the intake a bit up or down for best angle
            FoldoutState.SHOOT_MID to 45.0,
            FoldoutState.SHOOT_HIGH to 68.0,
            FoldoutState.INTAKE_DROP to 100.0,
            FoldoutState.INTAKE_GROUND to 230.0,
            FoldoutState.AMPLIFY to 109.0
        )

        /* The auto aims intake constants for optimal handoff, data in degrees of shooter per movement of Intake (TODO) */
        val neededPositionChanges = hashMapOf(
            FoldoutState.SHOOT_LOW to 0.0,
            FoldoutState.SHOOT_MID to 30.0,
            FoldoutState.SHOOT_HIGH to 50.0,
        )
    }

    object IntakeRollers {
        const val canID = 21

        const val beamBreakID = 2

        object Control {
            const val kP = 0.1
            const val kI = 0.0
            const val kD = 0.0
            const val kIz = 0.0
            const val kFF = 0.0
            const val kMinOutput = 0.0
            const val kMaxOutput = 0.0
        }

        enum class Speed {
            NEUTRAL, INTAKE, SHOOT, AMPLIFY
        }

        /* Speeds of the Intake Rollers (percent) */
        val speeds = hashMapOf(
            Speed.NEUTRAL to 0.0, // Apply a small amount of power to keep the note in place and centered
            Speed.INTAKE to 80.0,
            Speed.SHOOT to -100.0, // When Shooting this is the speed it pushes to the shooter
            Speed.AMPLIFY to -55.0
        )
    }

    object Climber {
        const val leftMotorID = 31
        const val rightMotorID = 32

        const val isLeftInverted = true
        const val isRightInverted = false

        const val maximumPosition = 114.0
        const val minimumPosition = 0.0

        const val theoreticalMax = 120.0 // The maximum position the climber can go to in rotations
    }
}
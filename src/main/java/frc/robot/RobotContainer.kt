// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.*
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.auto.AutoIntakeFromGround
import frc.robot.commands.auto.AutoIntakeLoadToShooter
import frc.robot.commands.climb.DefaultClimber
import frc.robot.commands.teleop.*
import frc.robot.commands.vision.PhotonPoseEstimator
import frc.robot.generated.TunerConstants
import frc.robot.subsystems.climb.Climber
import frc.robot.subsystems.foldouts.IntakeFoldout
import frc.robot.subsystems.light.Neopixels
import frc.robot.subsystems.rotation.IntakeRollers
import frc.robot.subsystems.vision.PhotonVision

import mu.KotlinLogging
import kotlin.math.absoluteValue

class RobotContainer {
    /* Set logger */
    private val textLogger = KotlinLogging.logger {}

    val m_Neopixels = Neopixels()

    private val MaxSpeed = Constants.Robot.maxSpeed
    private val MaxAngularRate = Constants.Robot.maxAngularSpeed

    private val joystick = CommandPS5Controller(0)
    private val climbJoystick = CommandPS5Controller(1)

    val drivetrain: CommandSwerveDrivetrain = TunerConstants.DriveTrain // My drivetrain

    private val drive = FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Open Loop Voltage control

    private val autoRotate = FieldCentricFacingAngle()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Open Loop Voltage control

    private val breakSwerve = SwerveDriveBrake()

    /* Feed Auto Command to Robot */
    val autonomousCommand : Command // So the sendable chooser built into PathPlanner can be used
        get() {
            return drivetrain.autoPath
        }

    val zeroButton = DigitalInput(9) // TODO: Add to config

    /* Subsystems */
    private val logger = Telemetry(MaxSpeed)
    private val m_Vision = PhotonVision(logger)

    val m_IntakeFoldout = IntakeFoldout()
    private val m_IntakeRollers = IntakeRollers()
    private val m_Climber = Climber()

    private val intakeLoadToShooterRunnable = Runnable {
        IntakeLoadToShooter(m_IntakeFoldout, m_IntakeRollers).schedule()
        m_Neopixels.updateState(Constants.Indicators.Neopixels.NeopixelStates.IDLE)
        m_Neopixels.chainBlinking = false
    }

    private fun configureSwerve() {
        drivetrain.defaultCommand = drivetrain.applyRequest {
            drive.withVelocityX(-joystick.leftY * MaxSpeed) // Drive forward with
                // negative Y (forward)
                .withVelocityY(-joystick.leftX * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.rightX * MaxAngularRate)
        }.withName("Free Driving") // Drive counterclockwise with negative X (left)

        // reset the field-centric heading on left bumper press
        joystick.L1().onTrue(drivetrain.runOnce { drivetrain.seedFieldRelative() })

        joystick.R2().whileTrue(
            drivetrain.applyRequest {
                drive.withVelocityX(-joystick.leftY * (MaxSpeed * .7)) // Drive forward with
                    // negative Y (forward)
                    .withVelocityY(-joystick.leftX * (MaxSpeed * .7)) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.rightX * MaxAngularRate)
            }.withName("Fast Driving") // Drive counterclockwise with negative X (left)
        )

        joystick.L2().whileTrue(
            drivetrain.applyRequest {
                drive.withVelocityX(-joystick.leftY * (MaxSpeed * .4)) // Drive forward with
                    // negative Y (forward)
                    .withVelocityY(-joystick.leftX * (MaxSpeed * .4)) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.rightX * MaxAngularRate)
            }.withName("Slow Driving") // Drive counterclockwise with negative X (left)
        )

        drivetrain.registerTelemetry { state: SwerveDriveState? ->
            logger.telemeterize(
                state!!
            )
        }
    }

    private fun configureButtons() {
        // Originally commands would not be timed and would end when its reached a setpoint, but we were messing with the controller and other stuff so didnt have time

        joystick.circle().toggleOnTrue( /* Intake from Ground, then load to shooter */
            IntakeFromGround(m_IntakeFoldout, m_IntakeRollers).alongWith(InstantCommand({
                m_Neopixels.updateState(Constants.Indicators.Neopixels.NeopixelStates.INTAKING)
                m_Neopixels.chainBlinking = true
            })).withName("IntakeFromGround Group").finallyDo(
                intakeLoadToShooterRunnable
            )
        )

        joystick.triangle().toggleOnTrue( /* Intake from Source, then load to shooter */
            SequentialCommandGroup(
                IntakeFromSource(m_IntakeFoldout, m_IntakeRollers).alongWith(InstantCommand({
                    m_Neopixels.updateState(Constants.Indicators.Neopixels.NeopixelStates.INTAKING)
                    m_Neopixels.chainBlinking = true
                }))
            ).finallyDo(
                intakeLoadToShooterRunnable
            )
        )

        joystick.square().onTrue( /* Shoot into the amp */
            SequentialCommandGroup(
                ParallelRaceGroup(
                    ShootToAmp(m_IntakeFoldout, m_IntakeRollers),
                    WaitCommand(.9)
                ).finallyDo(Runnable {
                    m_Neopixels.chainBlinking = true
                }),
                WaitCommand(.75)
            ).alongWith(InstantCommand({
                m_Neopixels.updateState(Constants.Indicators.Neopixels.NeopixelStates.SHOOTING)
            })).finallyDo(
                intakeLoadToShooterRunnable
            ).withName("ShootToAmp Group")
        )

        joystick.cross().onTrue(
            SequentialCommandGroup(
                ParallelRaceGroup(
                    ShootToSpeaker(m_IntakeFoldout, m_IntakeRollers),
                    WaitCommand(0.4)
                ).finallyDo(Runnable {
                    m_Neopixels.chainBlinking = true
                }),
                WaitCommand(0.4)
            ).alongWith(InstantCommand({
                m_Neopixels.updateState(Constants.Indicators.Neopixels.NeopixelStates.SHOOTING)
            })).finallyDo(
                intakeLoadToShooterRunnable
            ).withName("ShootToSpeaker Group")
        )

        joystick.povDown().whileTrue(
            drivetrain.applyRequest { breakSwerve }.alongWith(InstantCommand({
                m_Neopixels.chainBlinking = true
                m_Neopixels.updateState(Constants.Indicators.Neopixels.NeopixelStates.BREAKING)
            })).finallyDo(Runnable {
                m_Neopixels.chainBlinking = false
                m_Neopixels.updateState(Constants.Indicators.Neopixels.NeopixelStates.IDLE)
            })
        )

        Trigger { /* Zero button (if positions change from starting up to before a match) */
            !zeroButton.get()
        }.debounce(0.1).onTrue(
                m_IntakeFoldout.zeroEncoder()
                .finallyDo(Runnable {
                    textLogger.info { "Zeroing shooter via button" }
                }
            )
        )
    }

    private fun configureClimber() {
        m_Climber.defaultCommand = DefaultClimber(
            m_Climber,
            { if (climbJoystick.leftY.absoluteValue > 0.1) climbJoystick.leftY * -1 else 0.0 },
            { if (climbJoystick.rightY.absoluteValue > 0.1) climbJoystick.rightY * -1 else 0.0 }
        )
    }

    private fun configureEnhancers(){
        /* Automatic Aim */
        autoRotate.HeadingController = PhoenixPIDController(6.5, 0.0, 0.0)
        autoRotate.HeadingController.enableContinuousInput(-Math.PI, Math.PI)

//        if (Constants.supportEnhancers) {
//            joystick.R1().toggleOnTrue(
//                ParallelCommandGroup(
//                    drivetrain.applyRequest {
//                        autoRotate.withVelocityX(-joystick.leftY * MaxSpeed) // Drive forward with
//                            // negative Y (forward)
//                            .withVelocityY(-joystick.leftX * MaxSpeed) // Drive left with negative X (left)
//                            .withTargetDirection(
//                                Rotation2d.fromDegrees(
//                                    Tools.calculateFacingAngle(
//                                        if (DriverStation.getAlliance()
//                                                .get() == DriverStation.Alliance.Red
//                                        ) Constants.Enhancers.redScorePosition
//                                        else Constants.Enhancers.blueScorePosition,
//                                        logger.position
//                                    )
//                                )
//                            )
//                    }
//                ).withName("AutoAim")
//            )
//        } else {
//            textLogger.info { "Auto Aiming is OFF" }
//        }

        // Default stuff

//        joystick.R1().whileTrue(
//            drivetrain.applyRequest {
//                autoRotate.withVelocityX(-joystick.leftY * MaxSpeed) // Drive forward with
//                    // negative Y (forward)
//                    .withVelocityY(-joystick.leftX * MaxSpeed) // Drive left with negative X (left)
//                    .withTargetDirection(
//                        Rotation2d.fromDegrees(
//                            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) 90.0
//                            else 270.0
//                        )
//                    )
//            }.withName("Auto Heading to Amp")
//        )
    }

    private fun configureAuto() {
        NamedCommands.registerCommand("IntakeFromGround", AutoIntakeFromGround(m_IntakeFoldout, m_IntakeRollers))
        NamedCommands.registerCommand("IntakeLoadToShooter", AutoIntakeLoadToShooter(m_IntakeFoldout, m_IntakeRollers))

        NamedCommands.registerCommand("Amplify",
            SequentialCommandGroup(
                ParallelRaceGroup(
                    ShootToAmp(m_IntakeFoldout, m_IntakeRollers),
                    WaitCommand(.8)
                ).finallyDo(Runnable {
                    m_Neopixels.chainBlinking = true
                }),
                WaitCommand(1.0)
            ).alongWith(InstantCommand({
                m_Neopixels.updateState(Constants.Indicators.Neopixels.NeopixelStates.SHOOTING)
                m_Neopixels.chainBlinking = false
            })).finallyDo(Runnable {
                m_Neopixels.updateState(Constants.Indicators.Neopixels.NeopixelStates.AUTO)
            }).withName("ShootToAmp Auto Group")
        )

        NamedCommands.registerCommand("Poop",
                SequentialCommandGroup(
                    ParallelRaceGroup(
                        ShootToSpeaker(m_IntakeFoldout, m_IntakeRollers),
                        WaitCommand(0.2)
                    ).finallyDo(Runnable {
                        m_Neopixels.chainBlinking = true
                    }),
                    WaitCommand(0.4)
                ).alongWith(InstantCommand({
                    m_Neopixels.updateState(Constants.Indicators.Neopixels.NeopixelStates.SHOOTING)
                    m_Neopixels.chainBlinking = false
                })).finallyDo(
                    intakeLoadToShooterRunnable
                ).withName("ShootToSpeaker Group")
        )

        NamedCommands.registerCommand("Break", ParallelDeadlineGroup( // Breaks for 5 seconds
            WaitCommand(5.0),
            drivetrain.applyRequest { breakSwerve },
            InstantCommand({
                m_Neopixels.updateState(Constants.Indicators.Neopixels.NeopixelStates.BREAKING)
                m_Neopixels.chainBlinking = true
            })      ).finallyDo(Runnable { m_Neopixels.chainBlinking = false }))

        drivetrain.configurePathPlanner()
    }

    private fun configureVision() {
        if (Constants.Vision.enable) {
            PhotonPoseEstimator(drivetrain, logger, m_Vision).schedule()
        }
    }

    private fun configureRobotTools() {
        textLogger.info { "Robot tools are on!" }

        SmartDashboard.putData(CommandScheduler.getInstance())

        SmartDashboard.putData("Tools/IntakeFromGround", IntakeFromGround(m_IntakeFoldout, m_IntakeRollers))
        SmartDashboard.putData("Tools/IntakeFromSource", IntakeFromSource(m_IntakeFoldout, m_IntakeRollers))
        SmartDashboard.putData("Tools/Load", IntakeLoadToShooter(m_IntakeFoldout, m_IntakeRollers))
        SmartDashboard.putData("Tools/ShootToAmp", ShootToAmp(m_IntakeFoldout, m_IntakeRollers))

        SmartDashboard.putData("Tools/Intake Foldout Command", m_IntakeFoldout)
        SmartDashboard.putData("Tools/Intake Rollers", m_IntakeRollers)
    }

    private fun configureInternalTools() {
        SmartDashboard.putData("IntakeFoldout/Reset Encoder", m_IntakeFoldout.zeroEncoder())
        SmartDashboard.putData("Internals/Climber Retract", m_Climber.setClimberToMaxPositions())
        SmartDashboard.putData("Internals/Climber Uncap", m_Climber.setDisableSoftLimits())
    }

    init {
        configureVision()
        configureAuto()
        configureSwerve()
        configureEnhancers()
        configureButtons()
        configureClimber()

        if (Constants.isDebug) {
            configureRobotTools()
        }

        configureInternalTools()
    }
}
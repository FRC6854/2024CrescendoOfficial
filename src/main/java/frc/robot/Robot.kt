// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

class Robot : TimedRobot() {
    private var m_autonomousCommand: Command? = null
    private var m_robotContainer: RobotContainer? = null
    override fun robotInit() {
        m_robotContainer = RobotContainer()

        m_robotContainer!!.drivetrain.daqThread.setThreadPriority(99)

        HAL.report(FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin) // Flex on the FTAs
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        // Ignore this, I might do fancy stuff here someday
    }

    override fun disabledInit() {
        m_robotContainer!!.m_Neopixels.updateState(Constants.Indicators.Neopixels.NeopixelStates.DEAD)
        m_robotContainer!!.m_Neopixels.chainBlinking = false
    }
    override fun disabledPeriodic() {}
    override fun disabledExit() {}
    override fun autonomousInit() {
        m_robotContainer!!.m_Neopixels.updateState(Constants.Indicators.Neopixels.NeopixelStates.AUTO)
        m_robotContainer!!.m_Neopixels.chainBlinking = true

        m_autonomousCommand = m_robotContainer!!.autonomousCommand
        if (m_autonomousCommand != null) {
            m_autonomousCommand!!.schedule()
        }
    }

    override fun autonomousPeriodic() {}
    override fun autonomousExit() {
        m_robotContainer!!.m_Neopixels.chainBlinking = false
    }
    override fun teleopInit() {
        m_robotContainer!!.m_Neopixels.updateState(Constants.Indicators.Neopixels.NeopixelStates.IDLE)
        m_robotContainer!!.m_Neopixels.chainBlinking = false

        if (m_autonomousCommand != null) {
            m_autonomousCommand!!.cancel()
        }
    }

    override fun teleopPeriodic() {}
    override fun teleopExit() {}
    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {}
    override fun testExit() {}
    override fun simulationPeriodic() {}
}

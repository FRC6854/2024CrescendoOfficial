package frc.robot.subsystems.light

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.SerialPort
import frc.lib.light.LightActions
import frc.robot.Constants
import mu.KotlinLogging

class CameraLEDs() {
    private val logger = KotlinLogging.logger("VisionLEDs")
    private var connected = true;

    val serialPort = try { SerialPort(9600, SerialPort.Port.kUSB) } catch (e: Exception) { connected = false; null }

    init {
        if (Constants.Indicators.enableCameraLED) {
            if (connected) { // TODO: Add interactive error codes and blink codes
                logger.info { "Camera LEDs connected" }
                setAction(LightActions.IDLE)
            }
            else {
                logger.warn { "Camera LEDs not connected" }
            }
        } else {
            logger.info { "Camera LEDs disabled" }
        }
    }

    fun setAction(action: LightActions) {
        if (connected) {
            serialPort!!.writeString(action.toString())
        }
    }

    private fun sendString(string: String) {
        if (connected) {
            serialPort!!.writeString(string)

            serialPort.flush()

            serialPort.readString()
        }
    }
}

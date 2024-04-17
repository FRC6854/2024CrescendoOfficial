package frc.robot.subsystems.light

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import frc.lib.light.NeopixelAnimations
import frc.robot.Constants
import frc.lib.light.NeopixelSection

import mu.KotlinLogging

class Neopixels : SubsystemBase() {
    private val logger = KotlinLogging.logger("Neopixels")
    private var sections : Array<NeopixelSection> = Constants.Indicators.Neopixels.sections
    private var buffer: AddressableLEDBuffer = AddressableLEDBuffer(Constants.Indicators.Neopixels.numLEDs)
    private var led : AddressableLED = AddressableLED(Constants.Indicators.Neopixels.neopixelPort)

    private var relativeTick : Int = 0 // Used for animations, goes up to 500 then resets to 0
    private var started : Boolean = false

    var chainBlinking = false

    init {
        logger.info { "Neopixels initialized" }

        start()
    }

    private fun start() {
        if (Constants.Indicators.Neopixels.enabled) {
            logger.info { "Neopixels enabled" }
            started = true
            led.setLength(Constants.Indicators.Neopixels.numLEDs)
            led.start()

            updateState(Constants.Indicators.Neopixels.NeopixelStates.IDLE)
        } else {
            logger.info { "Neopixels disabled" }
        }
    }

    override fun periodic() {
        if (!started) return // If not started then don't do anything

        if (chainBlinking) {
            if (relativeTick % 10 > 5) {
                val tempBuffer = AddressableLEDBuffer(Constants.Indicators.Neopixels.numLEDs)
                for (i in 0 until Constants.Indicators.Neopixels.numLEDs) {
                    tempBuffer.setRGB(i, 200, 200, 200)
                }
                if (relativeTick % 2 == 0) led.setData(tempBuffer)
            } else {
                if (relativeTick % 2 == 0) publishBuffer()
            }
        } else {
            if (relativeTick % 2 == 0) publishBuffer()
        }

        relativeTick = (relativeTick + 1) % 161
    }

    private fun publishBuffer() {
        led.setData(buffer)
    }

    fun updateState(state : Constants.Indicators.Neopixels.NeopixelStates) {
        for (section in sections) {
            setSection(section, state)
        }
    }

    private fun setSection(section: NeopixelSection, cState: Constants.Indicators.Neopixels.NeopixelStates) {
        if (section.desiredState[cState] == null) return // If the desired state is null then don't do anything
        for (i in section.start..section.end) {
            buffer.setRGB(i,
                (section.desiredState[cState]!!.red * .7).toInt(),
                (section.desiredState[cState]!!.green * .7).toInt(),
                (section.desiredState[cState]!!.blue * .7).toInt()
            )
        }
    }
}
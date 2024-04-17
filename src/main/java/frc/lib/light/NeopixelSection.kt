package frc.lib.light

import frc.lib.light.NeopixelAnimations
import frc.robot.Constants.Indicators.Neopixels.NeopixelStates

data class NeopixelSection(
    val start: Int,
    val end: Int,
    val name: String,
    val desiredState: HashMap<NeopixelStates, Color>,
    var blacklistedAnimations: List<NeopixelAnimations>? = null // This is not required to be set, the subsystem will handle it
) {
    fun size(): Int {
        return end - start
    }
}
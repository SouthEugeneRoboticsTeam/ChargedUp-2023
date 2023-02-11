package org.sert2521.chargedup2023.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.chargedup2023.ElectronicIDs
import org.sert2521.chargedup2023.PhysicalConstants

enum class LEDSides {
    RIGHT,
    LEFT
}

object LEDs : SubsystemBase() {
    private val rightLEDs = AddressableLED(ElectronicIDs.ledRight)
    private val leftLEDS = AddressableLED(ElectronicIDs.ledLeft)

    private var rightBuffer = AddressableLEDBuffer(PhysicalConstants.ledRightLength)
    private var leftBuffer = AddressableLEDBuffer(PhysicalConstants.ledLeftLength)

    init {
        rightLEDs.setLength(PhysicalConstants.ledLeftLength)
        rightLEDs.start()

        leftLEDS.setLength(PhysicalConstants.ledRightLength)
        leftLEDS.start()

        update()
    }

    fun setLEDRGB(side: LEDSides, i: Int, r: Int, g: Int, b: Int) {
        when (side) {
            LEDSides.RIGHT -> rightBuffer.setRGB(i, r, g, b)
            LEDSides.LEFT -> leftBuffer.setRGB(i, r, g, b)
        }
    }

    fun setLEDHSV(side: LEDSides, i: Int, h: Int, s: Int, v: Int) {
        when (side) {
            LEDSides.RIGHT -> rightBuffer.setHSV(i, h, s, v)
            LEDSides.LEFT -> leftBuffer.setHSV(i, h, s, v



            )
        }
    }

    fun update() {
        rightLEDs.setData(rightBuffer)
        leftLEDS.setData(leftBuffer)
    }

    fun reset() {
        rightBuffer = AddressableLEDBuffer(PhysicalConstants.ledRightLength)
        leftBuffer = AddressableLEDBuffer(PhysicalConstants.ledLeftLength)
        update()
    }
}

package org.sert2521.chargedup2023.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.chargedup2023.ConfigConstants
import org.sert2521.chargedup2023.ElectronicIDs
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.commands.LedIdle

object LEDs : SubsystemBase() {
    private val ledStrip = AddressableLED(ElectronicIDs.ledId)
    private var ledBuffer = AddressableLEDBuffer(PhysicalConstants.ledLength)
    init {
        ledStrip.setLength(PhysicalConstants.ledLength)
        ledStrip.start()

        defaultCommand = LedIdle()
    }

    fun setLEDRGB(i: Int, r: Int, g: Int, b: Int) {
        ledBuffer.setRGB(i, r, g, b)
    }

    fun setLEDHSV(i: Int, h: Int, s: Int, v: Int) {
        ledBuffer.setHSV(i, h, s, v)

    }

    fun setAllLEDRGB(r: Int, g: Int, b: Int){
        for (i in 0..PhysicalConstants.ledLength-1) {
            setLEDRGB(i, r, g, b)
        }
    }

    fun setAllLEDHSV(h: Int, s: Int, v: Int){
        for (i in 0..PhysicalConstants.ledLength-1) {
            setLEDHSV(i, h, s, v)
        }
    }



    fun update() {
        if (RobotController.getBatteryVoltage() <= ConfigConstants.preLEDBrownOutVoltage) {
            setAllLEDRGB(0, 0, 0)
        }

        ledStrip.setData(ledBuffer)
    }

    override fun periodic(){
        update()
    }

    fun reset() {
        ledBuffer = AddressableLEDBuffer(PhysicalConstants.ledLength)
        update()
    }
}

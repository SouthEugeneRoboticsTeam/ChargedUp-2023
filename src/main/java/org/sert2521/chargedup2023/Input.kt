package org.sert2521.chargedup2023

import edu.wpi.first.wpilibj.XboxController

object Input {
    private val joystick = XboxController(0)

    fun getX(): Double {
        return -joystick.leftX
    }

    fun getY(): Double {
        return -joystick.leftY
    }

    fun getRot(): Double {
        return joystick.rightX
    }
}
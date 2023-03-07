package org.sert2521.chargedup2023.commands

import org.sert2521.chargedup2023.subsystems.Drivetrain

class VisionAlign : JoystickCommand() {

    init {
        addRequirements(Drivetrain)
    }

    override fun execute() {
        // Moving the y on the stick will affect the rate of change of the x
        val joystickData = readJoystick()


    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}
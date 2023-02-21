package org.sert2521.chargedup2023

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import org.sert2521.chargedup2023.commands.LedFlash
import org.sert2521.chargedup2023.commands.LedIdle
import org.sert2521.chargedup2023.subsystems.LEDSides
import org.sert2521.chargedup2023.subsystems.LEDs

object Input {
    private val driverController = XboxController(0)

    private val gunnerController = Joystick(1)

    private val intakeSetOne = JoystickButton(driverController, 3)

    private val intakeSetTwo = JoystickButton(driverController, 2)

    private val outtake = JoystickButton(driverController, 1)

    private val ledCube = JoystickButton(gunnerController, 3)

    private val ledCone = JoystickButton(gunnerController, 4)

    private val currentLedPatter = 1


    init {


        var currentCubePattern: LedFlash = LedFlash(PhysicalConstants.ledPurpleHSV[0], PhysicalConstants.ledPurpleHSV[1], PhysicalConstants.ledPurpleHSV[2])
        var currentConePattern: LedFlash = LedFlash(PhysicalConstants.ledYellowHSV[0], PhysicalConstants.ledYellowHSV[1], PhysicalConstants.ledYellowHSV[2])
        var ledRainbow: LedIdle = LedIdle()


        ledCube.toggleOnTrue(currentCubePattern)


        ledCone.toggleOnTrue(currentConePattern)



    }
}
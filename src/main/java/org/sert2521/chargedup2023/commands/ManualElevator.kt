package org.sert2521.chargedup2023.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.DemoConstants
import org.sert2521.chargedup2023.Input
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Elevator
import kotlin.math.*

class ManualElevator : CommandBase() {
    init {
        addRequirements(Elevator)
    }

    override fun execute() {
        val extensionMeasure = Elevator.extensionMeasure()
        val angleWrapMeasure = Elevator.angleWrapMeasure()

        val angleG = cos(angleWrapMeasure) * (TunedConstants.elevatorAngleG + extensionMeasure * TunedConstants.elevatorAngleGPerMeter)
        Elevator.setAngle(Input.getAngle() * DemoConstants.angleMultiplier + angleG)

        val extensionG = sin(angleWrapMeasure) * (TunedConstants.elevatorExtensionG)
        Elevator.setExtend(Input.getExtend() * DemoConstants.extendMultiplier + extensionG)
    }

    override fun end(interrupted: Boolean) {
        Elevator.stop()
    }
}
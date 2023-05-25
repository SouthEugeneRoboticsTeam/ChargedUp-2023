package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.DemoConstants
import org.sert2521.chargedup2023.Input
import org.sert2521.chargedup2023.PhysicalConstants
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
        val extensionMultiplier = clamp((PhysicalConstants.elevatorExtensionTop - extensionMeasure) / (PhysicalConstants.elevatorExtensionTop - PhysicalConstants.elevatorExtensionBottom), DemoConstants.elevatorSlowWhenExtended, 1.0)
        Elevator.setAngle(Input.getAngle() * DemoConstants.angleMultiplier * extensionMultiplier + angleG)

        val extensionG = sin(angleWrapMeasure) * TunedConstants.elevatorExtensionG
        Elevator.setExtend(Input.getExtend() * DemoConstants.extendMultiplier + extensionG)
    }

    override fun end(interrupted: Boolean) {
        Elevator.stop()
    }
}
package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.DemoConstants
import org.sert2521.chargedup2023.Input
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Elevator
import kotlin.math.*

class ManualElevator : CommandBase() {
    private val angleHomePID = PIDController(TunedConstants.elevatorAngleHomeP, TunedConstants.elevatorAngleHomeI, TunedConstants.elevatorAngleHomeD)

    init {
        addRequirements(Elevator)
    }

    override fun execute() {
        val angleWrapMeasure = Elevator.angleWrapMeasure()
        val extensionG = sin(angleWrapMeasure) * TunedConstants.elevatorExtensionG
        val extendInput = Input.getExtend()
        Elevator.setExtend(Input.getExtend() * DemoConstants.extendMultiplier + extensionG)

        val angleInput = if (abs(extendInput) > DemoConstants.autoHomeTrigger) {
            if (angleWrapMeasure < TunedConstants.elevatorExtensionMinAngleTarget) {
                1.0
            } else if (angleWrapMeasure > TunedConstants.elevatorExtensionMaxAngleTarget) {
                -1.0
            } else {
                Input.getAngle()
            }
        } else {
            Input.getAngle()
        }

        val extensionMeasure = Elevator.extensionMeasure()
        val angleG = cos(angleWrapMeasure) * (TunedConstants.elevatorAngleG + extensionMeasure * TunedConstants.elevatorAngleGPerMeter)
        val extensionMultiplier = clamp((PhysicalConstants.elevatorExtensionTop - extensionMeasure) / (PhysicalConstants.elevatorExtensionTop - PhysicalConstants.elevatorExtensionBottom), DemoConstants.elevatorSlowWhenExtended, 1.0)
        Elevator.setAngle(angleInput * DemoConstants.angleMultiplier * extensionMultiplier + angleG)
    }

    override fun end(interrupted: Boolean) {
        Elevator.stop()
    }
}
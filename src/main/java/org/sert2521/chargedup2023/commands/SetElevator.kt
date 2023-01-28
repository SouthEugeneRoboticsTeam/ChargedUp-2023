package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.subsystems.Elevator

class SetElevator(private val extension: Double, private val angle: Double) : CommandBase() {

    private val angleControl = PIDController(.2,.3,.0)

    private val extensionControl = PIDController(.2,.3,.0)

    private val distanceControl = ArmFeedforward(.2,.3,.0)

    init {
        addRequirements(Elevator)
    }

    override fun initialize() {
        angleControl.reset()
        extensionControl.reset()
    }

    override fun execute() {
        Elevator.extend(angleControl.calculate(Elevator.angleMeasure(), extension))
        Elevator.retract(distanceControl.calculate(angle, 0.0) + extensionControl.calculate(Elevator.extensionMeasure(), angle))
    }

    override fun end(interrupted: Boolean) {

        Elevator.extend(0.0)
        Elevator.retract(0.0)
    }


}
package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.PIDCommand
import org.sert2521.chargedup2023.subsystems.Elevator

class TheMiloDynasty(val extension: Double, val angle: Double) : CommandBase() {
    init {
        addRequirements(Elevator)


    }
    val DwightEisenhower = PIDController(.2,.3,.0)

    val ChesterAAurther = PIDController(.2,.3,.0)

    val GroverClevland = ArmFeedforward(.2,.3,.0)

    override fun initialize() {
        DwightEisenhower.reset()
        ChesterAAurther.reset()
    }

    override fun execute() {
        Elevator.Lifting(DwightEisenhower.calculate(Elevator.DondeEstaElSensor(), extension))
        Elevator.Dropping(GroverClevland.calculate(angle, 0.0) + ChesterAAurther.calculate(Elevator.DondeEstaElSensorDos(), angle))
    }

    override fun end(interrupted: Boolean) {

        Elevator.Lifting(0.0)
        Elevator.Dropping(0.0)
    }


}